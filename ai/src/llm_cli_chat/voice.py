from __future__ import annotations

import re
import shutil
import signal
import subprocess
from collections.abc import Callable
from dataclasses import dataclass, replace
from pathlib import Path
from threading import Lock, RLock, Thread, current_thread
from time import sleep
from typing import Any

import numpy as np


_tqdm_config_lock = Lock()
_tqdm_lock = RLock()
_tqdm_configured = False


@dataclass(frozen=True)
class VoiceInputConfig:
    voice_engine: str = "auto"
    whisper_model: str = "tiny.en"
    whisper_device: str = "auto"
    audio_input_device: int | None = None
    whisper_stream_command: str = "whisper-stream"
    whisper_stream_model: Path = Path("ggml-base.en.bin")
    whisper_stream_threads: int = 8
    whisper_stream_step_ms: int = 500
    whisper_stream_length_ms: int = 5000
    whisper_stream_keep_ms: int | None = None
    sample_rate: int = 16_000
    channels: int = 1
    realtime_interval_seconds: float = 0.5
    realtime_window_seconds: float = 5.0
    commit_interval_seconds: float = 5.0
    keep_overlap_seconds: float = 0.2
    min_realtime_seconds: float = 0.5
    speech_rms_threshold: float = 0.003
    send_realtime_on_stop: bool = True
    language: str | None = None


@dataclass(frozen=True)
class AudioInputDevice:
    index: int
    name: str
    max_input_channels: int
    default_samplerate: float | None = None
    is_default: bool = False


class VoiceInputController:
    """Records on demand and transcribes speech with Whisper in near real time."""

    def __init__(
        self,
        config: VoiceInputConfig,
        *,
        model: Any | None = None,
        on_ready: Callable[[str], None],
        on_recording_started: Callable[[], None],
        on_realtime_transcript: Callable[[str], None],
        on_recording_stopped: Callable[[], None],
        on_final_transcript: Callable[[str], None],
        on_error: Callable[[str], None],
    ) -> None:
        self.config = config
        self._on_ready = on_ready
        self._on_recording_started = on_recording_started
        self._on_realtime_transcript = on_realtime_transcript
        self._on_recording_stopped = on_recording_stopped
        self._on_final_transcript = on_final_transcript
        self._on_error = on_error

        self._engine = resolve_voice_engine(config)
        self._stream: Any | None = None
        self._stream_process: subprocess.Popen[str] | None = None
        self._stream_threads: list[Thread] = []
        self._stream_committed_transcript = ""
        self._stream_draft_transcript = ""
        self._stream_stderr = ""
        self._model: Any | None = model

        self._frames: list[np.ndarray] = []
        self._committed_transcript = ""
        self._live_transcript = ""
        self._last_commit_sample = 0
        self._started = False
        self._recording = False
        self._session_id = 0
        self._stopped = False

        self._recording_lock = Lock()
        self._model_lock = Lock()
        self._transcribe_lock = Lock()

    def start(self) -> None:
        if self._started:
            return

        self._started = True

        if self._engine == "whisper-stream":
            self._on_ready("Press Space to start recording.")
        elif self._model is None:
            self._on_ready("Press Space to start recording. Loading Whisper model...")
            Thread(target=self._warm_whisper_model, daemon=True).start()
        else:
            self._on_ready("Press Space to start recording.")

    def stop(self) -> None:
        self._stopped = True
        self._finish_recording(send_final=False, notify=False)

    def toggle_recording(self) -> None:
        if self._stopped:
            return

        if self.is_recording:
            self._finish_recording(send_final=True)
            return

        self._start_recording()

    def set_audio_input_device(self, audio_input_device: int | None) -> bool:
        with self._recording_lock:
            if self._recording:
                return False

            self.config = replace(self.config, audio_input_device=audio_input_device)
            return True

    @property
    def is_recording(self) -> bool:
        with self._recording_lock:
            return self._recording

    def _warm_whisper_model(self) -> None:
        try:
            self._load_model()
        except Exception as exc:  # noqa: BLE001 - model download/load failures are runtime setup issues.
            self._on_error(f"Whisper model could not load: {exc}")
            return

        if not self._stopped:
            self._on_ready("Press Space to start recording.")

    def _start_recording(self) -> None:
        if self._engine == "whisper-stream":
            self._start_whisper_stream_recording()
            return

        self._start_python_recording()

    def _finish_recording(self, *, send_final: bool, notify: bool = True) -> None:
        if self._engine == "whisper-stream":
            self._finish_whisper_stream_recording(send_final=send_final, notify=notify)
            return

        self._finish_python_recording(send_final=send_final, notify=notify)

    def _start_python_recording(self) -> None:
        with self._recording_lock:
            if self._recording or self._stopped:
                return

            self._frames = []
            self._committed_transcript = ""
            self._live_transcript = ""
            self._last_commit_sample = 0
            self._recording = True
            self._session_id += 1
            session_id = self._session_id

        try:
            sd = _load_sounddevice()

            stream = sd.InputStream(
                samplerate=self.config.sample_rate,
                channels=self.config.channels,
                device=self.config.audio_input_device,
                dtype="float32",
                callback=self._audio_callback,
            )
            stream.start()
        except Exception as exc:  # noqa: BLE001 - microphone permission/device errors should stay visible.
            with self._recording_lock:
                self._recording = False
                self._frames = []
            self._on_error(f"Microphone input could not start: {exc}")
            return

        close_stream = False
        with self._recording_lock:
            if self._recording and session_id == self._session_id:
                self._stream = stream
            else:
                close_stream = True

        if close_stream:
            try:
                stream.stop()
                stream.close()
            except Exception as exc:  # noqa: BLE001 - report audio cleanup failures.
                self._on_error(f"Microphone input could not stop cleanly: {exc}")
            return

        self._on_recording_started()
        Thread(target=self._realtime_transcription_loop, args=(session_id,), daemon=True).start()

    def _finish_python_recording(self, *, send_final: bool, notify: bool = True) -> None:
        with self._recording_lock:
            if not self._recording:
                return

            self._recording = False
            session_id = self._session_id
            frames = list(self._frames)
            committed_transcript = self._committed_transcript
            live_transcript = self._live_transcript
            realtime_transcript = join_transcript_segments(committed_transcript, live_transcript)
            last_commit_sample = self._last_commit_sample
            total_samples = sum(frame.size for frame in frames)
            self._frames = []
            stream = self._stream
            self._stream = None

        if stream is not None:
            try:
                stream.stop()
                stream.close()
            except Exception as exc:  # noqa: BLE001 - closing audio streams can surface driver errors.
                self._on_error(f"Microphone input could not stop cleanly: {exc}")

        if notify:
            self._on_recording_stopped()

        if send_final:
            uncommitted_samples = max(0, total_samples - last_commit_sample)
            min_samples = int(self.config.sample_rate * self.config.min_realtime_seconds)
            can_send_realtime = bool(live_transcript) or uncommitted_samples < min_samples

            if self.config.send_realtime_on_stop and realtime_transcript and can_send_realtime:
                self._on_final_transcript(realtime_transcript)
                return

            Thread(
                target=self._finalize_transcription,
                args=(session_id, frames, committed_transcript, last_commit_sample),
                daemon=True,
            ).start()

    def _start_whisper_stream_recording(self) -> None:
        with self._recording_lock:
            if self._recording or self._stopped:
                return

            self._stream_committed_transcript = ""
            self._stream_draft_transcript = ""
            self._stream_stderr = ""
            self._stream_threads = []
            self._recording = True
            self._session_id += 1
            session_id = self._session_id

        command = build_whisper_stream_command(self.config)
        try:
            process = subprocess.Popen(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                bufsize=1,
            )
        except Exception as exc:  # noqa: BLE001 - keep launch failures visible in the TUI.
            with self._recording_lock:
                self._recording = False
                self._stream_process = None
            self._on_error(f"whisper-stream could not start: {exc}")
            return

        stdout_thread = Thread(
            target=self._read_whisper_stream_stdout,
            args=(session_id, process),
            daemon=True,
        )
        stderr_thread = Thread(
            target=self._read_whisper_stream_stderr,
            args=(session_id, process),
            daemon=True,
        )
        watcher_thread = Thread(
            target=self._watch_whisper_stream_process,
            args=(session_id, process),
            daemon=True,
        )

        with self._recording_lock:
            if not self._recording or session_id != self._session_id:
                should_close = True
            else:
                should_close = False
                self._stream_process = process
                self._stream_threads = [stdout_thread, stderr_thread, watcher_thread]

        if should_close:
            self._stop_whisper_stream_process(process)
            return

        self._on_recording_started()
        stdout_thread.start()
        stderr_thread.start()
        watcher_thread.start()

    def _finish_whisper_stream_recording(
        self,
        *,
        send_final: bool,
        notify: bool = True,
    ) -> None:
        with self._recording_lock:
            if not self._recording:
                return

            self._recording = False
            session_id = self._session_id
            process = self._stream_process
            threads = list(self._stream_threads)
            self._stream_process = None
            self._stream_threads = []

        if notify:
            self._on_recording_stopped()

        if process is not None:
            self._stop_whisper_stream_process(process)

        current = current_thread()
        for thread in threads:
            if thread is not current and thread.is_alive():
                thread.join(timeout=0.5)

        with self._recording_lock:
            if session_id != self._session_id:
                return
            transcript = current_whisper_stream_transcript(
                self._stream_committed_transcript,
                self._stream_draft_transcript,
            )

        if not send_final:
            return

        if transcript:
            self._on_final_transcript(transcript)
            return

        self._on_ready("No speech recognized. Press Space to try again.")

    def _stop_whisper_stream_process(self, process: subprocess.Popen[str]) -> None:
        if process.poll() is not None:
            return

        try:
            process.send_signal(signal.SIGINT)
            process.wait(timeout=2)
            return
        except subprocess.TimeoutExpired:
            pass
        except Exception:
            pass

        if process.poll() is not None:
            return

        try:
            process.terminate()
            process.wait(timeout=2)
            return
        except subprocess.TimeoutExpired:
            pass
        except Exception:
            pass

        if process.poll() is None:
            try:
                process.kill()
                process.wait(timeout=1)
            except Exception:
                pass

    def _read_whisper_stream_stdout(
        self,
        session_id: int,
        process: subprocess.Popen[str],
    ) -> None:
        if process.stdout is None:
            return

        buffer = ""
        while True:
            char = process.stdout.read(1)
            if char == "":
                break
            if char in "\r\n":
                self._consume_whisper_stream_text(
                    session_id,
                    buffer,
                    commit=char == "\n",
                )
                buffer = ""
                continue
            buffer += char

        self._consume_whisper_stream_text(session_id, buffer, commit=True)

    def _read_whisper_stream_stderr(
        self,
        session_id: int,
        process: subprocess.Popen[str],
    ) -> None:
        if process.stderr is None:
            return

        while True:
            line = process.stderr.readline()
            if line == "":
                break
            with self._recording_lock:
                if session_id != self._session_id:
                    return
                self._stream_stderr = (self._stream_stderr + line)[-4000:]

    def _watch_whisper_stream_process(
        self,
        session_id: int,
        process: subprocess.Popen[str],
    ) -> None:
        returncode = process.wait()
        with self._recording_lock:
            if (
                session_id != self._session_id
                or process is not self._stream_process
                or not self._recording
            ):
                return

            self._recording = False
            self._stream_process = None
            stderr = self._stream_stderr.strip()

        if returncode != 0:
            detail = f": {stderr}" if stderr else ""
            self._on_error(f"whisper-stream exited with code {returncode}{detail}")
        else:
            self._on_ready("Recording stopped. Press Space to start recording.")

    def _consume_whisper_stream_text(
        self,
        session_id: int,
        raw_text: str,
        *,
        commit: bool,
    ) -> None:
        text = clean_whisper_stream_text(raw_text)

        with self._recording_lock:
            if session_id != self._session_id:
                return
            old_transcript = current_whisper_stream_transcript(
                self._stream_committed_transcript,
                self._stream_draft_transcript,
            )
            (
                self._stream_committed_transcript,
                self._stream_draft_transcript,
            ) = update_whisper_stream_transcript(
                self._stream_committed_transcript,
                self._stream_draft_transcript,
                text,
                commit=commit,
            )
            transcript = current_whisper_stream_transcript(
                self._stream_committed_transcript,
                self._stream_draft_transcript,
            )
            if transcript == old_transcript:
                return

        self._on_realtime_transcript(transcript)

    def _audio_callback(
        self,
        indata: np.ndarray,
        _frames: int,
        _time_info: Any,
        status: Any,
    ) -> None:
        if status:
            self._on_error(f"Audio input warning: {status}")

        audio = indata.copy().reshape(-1).astype(np.float32)
        with self._recording_lock:
            if self._recording:
                self._frames.append(audio)

    def _realtime_transcription_loop(self, session_id: int) -> None:
        last_display_text = ""
        min_samples = int(self.config.sample_rate * self.config.min_realtime_seconds)
        commit_samples = max(
            min_samples,
            int(self.config.sample_rate * self.config.commit_interval_seconds),
        )
        keep_samples = int(self.config.sample_rate * self.config.keep_overlap_seconds)

        while self._is_recording_session(session_id):
            sleep(self.config.realtime_interval_seconds)
            audio = self._snapshot_audio(session_id)

            with self._recording_lock:
                committed_transcript = self._committed_transcript
                last_commit_sample = self._last_commit_sample

            uncommitted_samples = audio.size - last_commit_sample
            if uncommitted_samples < min_samples:
                continue

            chunk_start = max(0, last_commit_sample - keep_samples)
            chunk_audio = self._realtime_window(audio[chunk_start:])
            if not self._has_speech(chunk_audio):
                continue

            try:
                text = self._transcribe(chunk_audio)
            except Exception as exc:  # noqa: BLE001 - surface runtime speech errors in the TUI.
                self._on_error(f"Whisper transcription failed: {exc}")
                return

            live_transcript = new_transcript_suffix(committed_transcript, text)
            if not live_transcript:
                continue

            display_text = join_transcript_segments(committed_transcript, live_transcript)
            should_commit = uncommitted_samples >= commit_samples

            with self._recording_lock:
                if not self._recording or session_id != self._session_id:
                    return
                if should_commit:
                    self._committed_transcript = display_text
                    self._live_transcript = ""
                    self._last_commit_sample = audio.size
                else:
                    self._live_transcript = live_transcript

            if display_text != last_display_text:
                last_display_text = display_text
                self._on_realtime_transcript(display_text)

    def _finalize_transcription(
        self,
        session_id: int,
        frames: list[np.ndarray],
        committed_transcript: str = "",
        last_commit_sample: int = 0,
    ) -> None:
        audio = self._join_frames(frames)
        if audio.size == 0:
            if session_id == self._session_id:
                self._on_ready("No speech captured. Press Space to try again.")
            return

        if committed_transcript and last_commit_sample > 0:
            keep_samples = int(self.config.sample_rate * self.config.keep_overlap_seconds)
            chunk_start = max(0, last_commit_sample - keep_samples)
            audio = audio[chunk_start:]

        try:
            text = self._transcribe(audio)
        except Exception as exc:  # noqa: BLE001 - surface runtime speech errors in the TUI.
            self._on_error(f"Whisper transcription failed: {exc}")
            return

        text = join_transcript_segments(
            committed_transcript,
            new_transcript_suffix(committed_transcript, text),
        )

        if text:
            self._on_final_transcript(text)
            return

        if session_id == self._session_id:
            self._on_ready("No speech recognized. Press Space to try again.")

    def _is_recording_session(self, session_id: int) -> bool:
        with self._recording_lock:
            return self._recording and session_id == self._session_id

    def _snapshot_audio(self, session_id: int) -> np.ndarray:
        with self._recording_lock:
            if session_id != self._session_id:
                return np.array([], dtype=np.float32)
            return self._join_frames(self._frames)

    def _join_frames(self, frames: list[np.ndarray]) -> np.ndarray:
        if not frames:
            return np.array([], dtype=np.float32)
        return np.concatenate(frames).astype(np.float32, copy=False)

    def _realtime_window(self, audio: np.ndarray) -> np.ndarray:
        max_samples = int(self.config.sample_rate * self.config.realtime_window_seconds)
        if max_samples <= 0 or audio.size <= max_samples:
            return audio
        return audio[-max_samples:]

    def _has_speech(self, audio: np.ndarray) -> bool:
        if audio.size == 0:
            return False
        audio = audio.astype(np.float32, copy=False)
        rms = float(np.sqrt(np.mean(audio * audio)))
        return rms >= self.config.speech_rms_threshold

    def _load_model(self) -> Any:
        with self._model_lock:
            if self._model is None:
                self._model = load_whisper_model(
                    self.config.whisper_model,
                    device=self.config.whisper_device,
                )
            return self._model

    def _transcribe(self, audio: np.ndarray) -> str:
        configure_tqdm_for_textual()
        model = self._load_model()
        options: dict[str, Any] = {
            "fp16": uses_cuda(model),
            # Whisper uses verbose=False to enable tqdm progress bars, which
            # conflicts with Textual's terminal control.
            "verbose": None,
            "condition_on_previous_text": False,
            "temperature": 0.0,
            "without_timestamps": True,
        }
        language = self.config.language
        if language is None and self.config.whisper_model.endswith(".en"):
            language = "en"
        if language:
            options["language"] = language

        with self._transcribe_lock:
            result = model.transcribe(audio, **options)

        return str(result.get("text", "")).strip()


def load_whisper_model(model_name: str, *, device: str = "auto") -> Any:
    configure_tqdm_for_textual()
    import whisper

    return whisper.load_model(model_name, device=resolve_whisper_device(device))


def resolve_whisper_device(device: str) -> str:
    if device != "auto":
        return device

    import torch

    if torch.cuda.is_available():
        return "cuda"

    return "cpu"


def resolve_voice_engine(config: VoiceInputConfig) -> str:
    if config.voice_engine not in {"auto", "python", "whisper-stream"}:
        raise ValueError("voice_engine must be one of: auto, python, whisper-stream")

    if config.voice_engine != "auto":
        return config.voice_engine

    if whisper_stream_unavailable_reason(config) is None:
        return "whisper-stream"

    return "python"


def whisper_stream_unavailable_reason(config: VoiceInputConfig) -> str | None:
    if shutil.which(config.whisper_stream_command) is None:
        return f"{config.whisper_stream_command!r} was not found on PATH"

    model_path = Path(config.whisper_stream_model)
    if not model_path.exists():
        return f"model file {str(model_path)!r} does not exist"

    return None


def build_whisper_stream_command(config: VoiceInputConfig) -> list[str]:
    command = [
        config.whisper_stream_command,
        "-m",
        str(Path(config.whisper_stream_model)),
        "-t",
        str(config.whisper_stream_threads),
        "--step",
        str(config.whisper_stream_step_ms),
        "--length",
        str(config.whisper_stream_length_ms),
    ]
    if config.audio_input_device is not None:
        command.extend(["--capture", str(config.audio_input_device)])

    if config.whisper_stream_keep_ms is not None and config.whisper_stream_keep_ms >= 0:
        command.extend(["--keep", str(config.whisper_stream_keep_ms)])

    if config.language:
        command.extend(["--language", config.language])

    return command


def list_audio_input_devices() -> list[AudioInputDevice]:
    sd = _load_sounddevice()
    default_index = default_audio_input_device_index(sd)
    devices = []

    for index, device in enumerate(sd.query_devices()):
        max_input_channels = _device_int(device, "max_input_channels")
        if max_input_channels <= 0:
            continue

        name = str(device.get("name") or f"Input device {index}")
        samplerate = _device_float(device, "default_samplerate")
        devices.append(
            AudioInputDevice(
                index=index,
                name=name,
                max_input_channels=max_input_channels,
                default_samplerate=samplerate,
                is_default=index == default_index,
            )
        )

    return devices


def audio_input_device_label(device: AudioInputDevice) -> str:
    details = [f"{device.max_input_channels} ch"]
    if device.default_samplerate is not None:
        details.append(f"{device.default_samplerate:g} Hz")
    if device.is_default:
        details.append("default")

    return f"{device.index}: {device.name} ({', '.join(details)})"


def default_audio_input_device_index(sd: Any) -> int | None:
    try:
        default_device = sd.default.device
    except Exception:
        return None

    if isinstance(default_device, (list, tuple)):
        default_device = default_device[0] if default_device else None

    try:
        index = int(default_device)
    except (TypeError, ValueError):
        return None

    return index if index >= 0 else None


def _load_sounddevice() -> Any:
    import sounddevice as sd

    return sd


def _device_int(device: Any, key: str) -> int:
    try:
        return int(device.get(key) or 0)
    except (TypeError, ValueError, AttributeError):
        return 0


def _device_float(device: Any, key: str) -> float | None:
    try:
        value = device.get(key)
    except AttributeError:
        return None

    if value is None:
        return None

    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def uses_cuda(model: Any) -> bool:
    try:
        parameter = next(model.parameters())
    except StopIteration:
        return False

    return str(parameter.device).startswith("cuda")


_ANSI_RE = re.compile(r"\x1b\[[0-9;?]*[ -/]*[@-~]")
_TIMESTAMP_RE = re.compile(r"\[[^\]]*?-->\s*[^\]]*?\]\s*")
_STREAM_MARKER_RE = re.compile(r"^\[(?:Start speaking|BLANK_AUDIO)\]\s*", re.IGNORECASE)
_STATUS_PREFIXES = (
    "init:",
    "main:",
    "system_info:",
    "sampling:",
    "whisper_",
    "ggml_",
)


def clean_whisper_stream_text(raw_text: str) -> str:
    text = _ANSI_RE.sub("", raw_text)
    text = _TIMESTAMP_RE.sub("", text)
    text = _STREAM_MARKER_RE.sub("", text)
    text = text.replace("\x00", "").strip()
    if not text:
        return ""

    lowered = text.casefold()
    if lowered.startswith(_STATUS_PREFIXES):
        return ""

    return " ".join(text.split())


def merge_transcript_update(transcript: str, update: str) -> str:
    transcript = transcript.strip()
    update = update.strip()
    if not transcript:
        return update
    if not update:
        return transcript

    transcript_words = transcript.split()
    update_words = update.split()
    normalized_transcript = _normalized_words(transcript_words)
    normalized_update = _normalized_words(update_words)
    if _contains_word_sequence(normalized_transcript, normalized_update):
        return transcript

    max_overlap = min(len(transcript_words), len(update_words), 80)
    for overlap in range(max_overlap, 0, -1):
        left = normalized_transcript[-overlap:]
        right = normalized_update[:overlap]
        if _is_acceptable_overlap(left, right):
            merged_words = transcript_words[:-overlap] + update_words
            return " ".join(merged_words).strip()

    return join_transcript_segments(transcript, update)


def update_whisper_stream_transcript(
    committed: str,
    draft: str,
    update: str,
    *,
    commit: bool,
) -> tuple[str, str]:
    committed = committed.strip()
    draft = draft.strip()
    update = update.strip()

    if commit:
        if update:
            draft = update
        if draft:
            committed = merge_transcript_update(committed, draft)
        return committed, ""

    if not update:
        return committed, draft

    return committed, update


def current_whisper_stream_transcript(committed: str, draft: str) -> str:
    committed = committed.strip()
    draft = draft.strip()
    if not draft:
        return committed
    return merge_transcript_update(committed, draft)


def new_transcript_suffix(committed: str, draft: str) -> str:
    committed = committed.strip()
    draft = draft.strip()
    if not draft:
        return ""
    if not committed:
        return draft

    committed_words = committed.split()
    draft_words = draft.split()
    if not draft_words:
        return ""

    normalized_committed = _normalized_words(committed_words)
    normalized_draft = _normalized_words(draft_words)
    if _contains_word_sequence(normalized_committed, normalized_draft):
        return ""

    max_overlap = min(len(committed_words), len(draft_words))
    for overlap in range(max_overlap, 0, -1):
        if _words_equal(committed_words[-overlap:], draft_words[:overlap]):
            return " ".join(draft_words[overlap:]).strip()

    return draft


def join_transcript_segments(committed: str, draft: str) -> str:
    committed = committed.strip()
    draft = draft.strip()
    if not committed:
        return draft
    if not draft:
        return committed
    if draft[0] in ".,!?;:)]}":
        return f"{committed}{draft}"
    return f"{committed} {draft}"


def _words_equal(left: list[str], right: list[str]) -> bool:
    return _normalized_words(left) == _normalized_words(right)


def _normalized_words(words: list[str]) -> list[str]:
    return [word.strip(".,!?;:\"'()[]{}").casefold() for word in words if word.strip()]


def _contains_word_sequence(words: list[str], sequence: list[str]) -> bool:
    if not sequence:
        return False
    if len(sequence) > len(words):
        return False

    stop = len(words) - len(sequence) + 1
    return any(words[index : index + len(sequence)] == sequence for index in range(stop))


def _is_acceptable_overlap(left: list[str], right: list[str]) -> bool:
    if not left or len(left) != len(right):
        return False

    matches = sum(
        left_word == right_word for left_word, right_word in zip(left, right, strict=True)
    )
    if matches == len(left):
        return True

    overlap = len(left)
    if overlap < 3:
        return False

    required_matches = max(2, int(overlap * 0.67))
    anchored = left[0] == right[0] or left[-1] == right[-1]
    return anchored and matches >= required_matches


def configure_tqdm_for_textual() -> None:
    """Avoid tqdm creating multiprocessing locks while Textual owns the terminal."""
    global _tqdm_configured

    with _tqdm_config_lock:
        if _tqdm_configured:
            return

        from tqdm import tqdm

        tqdm.set_lock(_tqdm_lock)
        _tqdm_configured = True
