from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from threading import Lock, RLock, Thread
from time import sleep
from typing import Any

import numpy as np


_tqdm_config_lock = Lock()
_tqdm_lock = RLock()
_tqdm_configured = False


@dataclass(frozen=True)
class VoiceInputConfig:
    whisper_model: str = "tiny.en"
    whisper_device: str = "auto"
    sample_rate: int = 16_000
    channels: int = 1
    realtime_interval_seconds: float = 0.25
    realtime_window_seconds: float = 2.0
    commit_interval_seconds: float = 1.5
    keep_overlap_seconds: float = 0.2
    min_realtime_seconds: float = 0.5
    speech_rms_threshold: float = 0.003
    send_realtime_on_stop: bool = True
    language: str | None = None


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

        if self._model is None:
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
            import sounddevice as sd

            stream = sd.InputStream(
                samplerate=self.config.sample_rate,
                channels=self.config.channels,
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

    def _finish_recording(self, *, send_final: bool, notify: bool = True) -> None:
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


def uses_cuda(model: Any) -> bool:
    try:
        parameter = next(model.parameters())
    except StopIteration:
        return False

    return str(parameter.device).startswith("cuda")


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


def configure_tqdm_for_textual() -> None:
    """Avoid tqdm creating multiprocessing locks while Textual owns the terminal."""
    global _tqdm_configured

    with _tqdm_config_lock:
        if _tqdm_configured:
            return

        from tqdm import tqdm

        tqdm.set_lock(_tqdm_lock)
        _tqdm_configured = True
