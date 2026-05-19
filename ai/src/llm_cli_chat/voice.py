from __future__ import annotations

import re
import shutil
import subprocess
import tempfile
import wave
from collections.abc import Callable
from dataclasses import dataclass, replace
from pathlib import Path
from threading import Lock, RLock, Thread
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
    whisper_cli_command: str = "whisper-cli"
    whisper_cli_model: Path = Path("ggml-base.en.bin")
    whisper_cli_threads: int = 8
    whisper_stream_command: str | None = None
    whisper_stream_model: Path | None = None
    whisper_stream_threads: int | None = None
    whisper_stream_step_ms: int | None = None
    whisper_stream_length_ms: int | None = None
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

    def __post_init__(self) -> None:
        if self.whisper_stream_command is not None:
            object.__setattr__(self, "whisper_cli_command", self.whisper_stream_command)
        if self.whisper_stream_model is not None:
            object.__setattr__(self, "whisper_cli_model", self.whisper_stream_model)
        if self.whisper_stream_threads is not None:
            object.__setattr__(self, "whisper_cli_threads", self.whisper_stream_threads)


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

        if self._engine == "whisper-cli":
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
        if self._engine == "whisper-cli":
            self._start_whisper_cli_recording()
            return

        self._start_python_recording()

    def _finish_recording(self, *, send_final: bool, notify: bool = True) -> None:
        if self._engine == "whisper-cli":
            self._finish_whisper_cli_recording(send_final=send_final, notify=notify)
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
            except Exception as exc:  # noqa: BLE001
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

    def _start_whisper_cli_recording(self) -> None:
        with self._recording_lock:
            if self._recording or self._stopped:
                return

            self._frames = []
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
        except Exception as exc:  # noqa: BLE001 - keep launch failures visible in the TUI.
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

    def _finish_whisper_cli_recording(
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
            config = self.config
            frames = list(self._frames)
            self._frames = []
            stream = self._stream
            self._stream = None

        if stream is not None:
            try:
                stream.stop()
                stream.close()
            except Exception as exc:  # noqa: BLE001
                self._on_error(f"Microphone input could not stop cleanly: {exc}")

        if notify:
            self._on_recording_stopped()

        if not send_final:
            return

        Thread(
            target=self._finalize_whisper_cli_transcription,
            args=(session_id, frames, config),
            daemon=True,
        ).start()

    def _finalize_whisper_cli_transcription(
        self,
        session_id: int,
        frames: list[np.ndarray],
        config: VoiceInputConfig,
    ) -> None:
        audio = self._join_frames(frames)
        if audio.size == 0:
            if session_id == self._session_id:
                self._on_ready("No speech captured. Press Space to try again.")
            return

        if not self._has_speech(audio):
            if session_id == self._session_id:
                self._on_ready("No speech recognized. Press Space to try again.")
            return

        try:
            with tempfile.TemporaryDirectory(prefix="llm-chat-voice-") as temp_dir:
                audio_path = Path(temp_dir) / "recording.wav"
                write_wav_audio(audio_path, audio, sample_rate=config.sample_rate)
                transcript = self._transcribe_with_whisper_cli(audio_path, config)
        except Exception as exc:  # noqa: BLE001 - surface runtime speech errors in the TUI.
            self._on_error(f"whisper-cli transcription failed: {exc}")
            return

        if session_id != self._session_id:
            return

        if transcript:
            self._on_final_transcript(transcript)
            return

        self._on_ready("No speech recognized. Press Space to try again.")

    def _transcribe_with_whisper_cli(self, audio_path: Path, config: VoiceInputConfig) -> str:
        command = build_whisper_cli_command(config, audio_path)
        completed = subprocess.run(
            command,
            capture_output=True,
            text=True,
            check=False,
        )
        if completed.returncode != 0:
            detail = completed.stderr.strip() or completed.stdout.strip()
            if detail:
                raise RuntimeError(
                    f"{command[0]} exited with code {completed.returncode}: {detail}"
                )
            raise RuntimeError(f"{command[0]} exited with code {completed.returncode}")

        return clean_whisper_cli_text(
            "\n".join(
                part
                for part in (completed.stdout, completed.stderr)
                if part
            )
        )

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
    if config.voice_engine not in {"auto", "python", "whisper-cli", "whisper-stream"}:
        raise ValueError(
            "voice_engine must be one of: auto, python, whisper-cli, whisper-stream"
        )

    if config.voice_engine == "whisper-stream":
        return "whisper-cli"

    if config.voice_engine != "auto":
        return config.voice_engine

    if whisper_cli_unavailable_reason(config) is None:
        return "whisper-cli"

    return "python"


def whisper_cli_unavailable_reason(config: VoiceInputConfig) -> str | None:
    if shutil.which(config.whisper_cli_command) is None:
        return f"{config.whisper_cli_command!r} was not found on PATH"

    model_path = Path(config.whisper_cli_model)
    if not model_path.exists():
        return f"model file {str(model_path)!r} does not exist"

    return None


def whisper_stream_unavailable_reason(config: VoiceInputConfig) -> str | None:
    return whisper_cli_unavailable_reason(config)


def build_whisper_cli_command(
    config: VoiceInputConfig,
    audio_path: Path | None = None,
) -> list[str]:
    command = [
        config.whisper_cli_command,
        "-m",
        str(Path(config.whisper_cli_model)),
        "-t",
        str(config.whisper_cli_threads),
        "--no-timestamps",
    ]
    if config.language:
        command.extend(["--language", config.language])

    if audio_path is not None:
        command.extend(["-f", str(audio_path)])

    return command


def build_whisper_stream_command(
    config: VoiceInputConfig,
    audio_path: Path | None = None,
) -> list[str]:
    return build_whisper_cli_command(config, audio_path)


def write_wav_audio(audio_path: Path, audio: np.ndarray, *, sample_rate: int) -> None:
    clipped = np.clip(audio.astype(np.float32, copy=False), -1.0, 1.0)
    pcm = (clipped * np.iinfo(np.int16).max).astype(np.int16)

    with wave.open(str(audio_path), "wb") as wav_file:
        wav_file.setnchannels(1)
        wav_file.setsampwidth(np.dtype(np.int16).itemsize)
        wav_file.setframerate(sample_rate)
        wav_file.writeframes(pcm.tobytes())


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


def clean_whisper_cli_text(raw_text: str) -> str:
    lines = []
    for line in raw_text.replace("\r", "\n").splitlines():
        text = clean_whisper_cli_line(line)
        if text:
            lines.append(text)

    return " ".join(lines).strip()


def clean_whisper_cli_line(raw_text: str) -> str:
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


def clean_whisper_stream_text(raw_text: str) -> str:
    return clean_whisper_cli_text(raw_text)


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
