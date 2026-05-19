from __future__ import annotations

import tempfile
import unittest
from pathlib import Path
from unittest import mock

import numpy as np

from llm_cli_chat.voice import (
    AudioInputDevice,
    VoiceInputConfig,
    VoiceInputController,
    audio_input_device_label,
    build_whisper_cli_command,
    clean_whisper_cli_text,
    current_whisper_stream_transcript,
    list_audio_input_devices,
    merge_transcript_update,
    resolve_voice_engine,
    update_whisper_stream_transcript,
    write_wav_audio,
)


def mock_audio() -> np.ndarray:
    return np.array([0.0, 0.25, -0.25, 0.5], dtype=np.float32)


class VoiceHelperTests(unittest.TestCase):
    def test_build_whisper_cli_command_matches_default_settings(self) -> None:
        command = build_whisper_cli_command(
            VoiceInputConfig(whisper_cli_model=Path("ggml-base.en.bin")),
        )

        self.assertEqual(
            command,
            [
                "whisper-cli",
                "-m",
                "ggml-base.en.bin",
                "-t",
                "8",
                "--no-timestamps",
            ],
        )

    def test_build_whisper_cli_command_includes_audio_file_when_provided(self) -> None:
        command = build_whisper_cli_command(
            VoiceInputConfig(
                whisper_cli_model=Path("ggml-base.en.bin"),
            ),
            Path("recording.wav"),
        )

        self.assertIn("-f", command)
        self.assertEqual(command[command.index("-f") + 1], "recording.wav")

    def test_legacy_whisper_stream_config_fields_feed_whisper_cli_command(self) -> None:
        command = build_whisper_cli_command(
            VoiceInputConfig(
                whisper_stream_command="whisper-cli",
                whisper_stream_model=Path("ggml-base.en.bin"),
                whisper_stream_threads=8,
            ),
            Path("recording.wav"),
        )

        self.assertEqual(command[0], "whisper-cli")
        self.assertIn("ggml-base.en.bin", command)
        self.assertIn("8", command)
        self.assertIn("recording.wav", command)

    def test_python_recording_passes_selected_input_device_to_sounddevice(self) -> None:
        stream = mock.Mock()
        fake_sd = mock.Mock()
        fake_sd.InputStream.return_value = stream
        controller = VoiceInputController(
            VoiceInputConfig(voice_engine="python", audio_input_device=7),
            model=mock.Mock(),
            on_ready=mock.Mock(),
            on_recording_started=mock.Mock(),
            on_realtime_transcript=mock.Mock(),
            on_recording_stopped=mock.Mock(),
            on_final_transcript=mock.Mock(),
            on_error=mock.Mock(),
        )

        with mock.patch("llm_cli_chat.voice._load_sounddevice", return_value=fake_sd):
            controller._start_python_recording()
            controller._finish_python_recording(send_final=False, notify=False)

        fake_sd.InputStream.assert_called_once()
        self.assertEqual(fake_sd.InputStream.call_args.kwargs["device"], 7)

    def test_whisper_cli_recording_passes_selected_input_device_to_sounddevice(self) -> None:
        stream = mock.Mock()
        fake_sd = mock.Mock()
        fake_sd.InputStream.return_value = stream
        controller = VoiceInputController(
            VoiceInputConfig(voice_engine="whisper-cli", audio_input_device=7),
            model=mock.Mock(),
            on_ready=mock.Mock(),
            on_recording_started=mock.Mock(),
            on_realtime_transcript=mock.Mock(),
            on_recording_stopped=mock.Mock(),
            on_final_transcript=mock.Mock(),
            on_error=mock.Mock(),
        )

        with mock.patch("llm_cli_chat.voice._load_sounddevice", return_value=fake_sd):
            controller._start_whisper_cli_recording()
            controller._finish_whisper_cli_recording(send_final=False, notify=False)

        fake_sd.InputStream.assert_called_once()
        self.assertEqual(fake_sd.InputStream.call_args.kwargs["device"], 7)

    def test_list_audio_input_devices_filters_outputs_and_marks_default(self) -> None:
        fake_sd = mock.Mock()
        fake_sd.default.device = (2, 4)
        fake_sd.query_devices.return_value = [
            {"name": "Speaker", "max_input_channels": 0},
            {"name": "Backup Mic", "max_input_channels": 1, "default_samplerate": 44100},
            {"name": "Suit Mic", "max_input_channels": 2, "default_samplerate": 48000},
        ]

        with mock.patch("llm_cli_chat.voice._load_sounddevice", return_value=fake_sd):
            devices = list_audio_input_devices()

        self.assertEqual(
            devices,
            [
                AudioInputDevice(
                    index=1,
                    name="Backup Mic",
                    max_input_channels=1,
                    default_samplerate=44100.0,
                    is_default=False,
                ),
                AudioInputDevice(
                    index=2,
                    name="Suit Mic",
                    max_input_channels=2,
                    default_samplerate=48000.0,
                    is_default=True,
                ),
            ],
        )
        self.assertEqual(
            audio_input_device_label(devices[1]),
            "2: Suit Mic (2 ch, 48000 Hz, default)",
        )

    def test_clean_whisper_cli_text_removes_timestamps_and_status_lines(self) -> None:
        self.assertEqual(
            clean_whisper_cli_text("\x1b[32m[00:00:00.000 --> 00:00:01.000]  hello   world"),
            "hello world",
        )
        self.assertEqual(clean_whisper_cli_text("[Start speaking]  hello world"), "hello world")
        self.assertEqual(clean_whisper_cli_text("init: found 1 capture devices"), "")

    def test_write_wav_audio_writes_mono_pcm_segment(self) -> None:
        import wave

        with tempfile.TemporaryDirectory() as temp_dir:
            audio_path = Path(temp_dir) / "segment.wav"
            write_wav_audio(audio_path, mock_audio(), sample_rate=16_000)

            with wave.open(str(audio_path), "rb") as wav_file:
                self.assertEqual(wav_file.getnchannels(), 1)
                self.assertEqual(wav_file.getsampwidth(), 2)
                self.assertEqual(wav_file.getframerate(), 16_000)
                self.assertGreater(wav_file.getnframes(), 0)

    def test_merge_transcript_update_uses_overlaps_without_repeating_words(self) -> None:
        self.assertEqual(
            merge_transcript_update("go to the airlock", "the airlock and close the hatch"),
            "go to the airlock and close the hatch",
        )

    def test_merge_transcript_update_can_replace_recent_corrected_words(self) -> None:
        self.assertEqual(
            merge_transcript_update("set the osha", "set the oxygen pressure"),
            "set the oxygen pressure",
        )

    def test_stream_carriage_return_replaces_live_draft(self) -> None:
        committed, draft = update_whisper_stream_transcript(
            "",
            "",
            "turn left at",
            commit=False,
        )
        committed, draft = update_whisper_stream_transcript(
            committed,
            draft,
            "turn right at the airlock",
            commit=False,
        )

        self.assertEqual(
            current_whisper_stream_transcript(committed, draft),
            "turn right at the airlock",
        )

    def test_stream_newline_commits_live_draft(self) -> None:
        committed, draft = update_whisper_stream_transcript(
            "",
            "",
            "open the hatch",
            commit=False,
        )
        committed, draft = update_whisper_stream_transcript(committed, draft, "", commit=True)

        self.assertEqual(committed, "open the hatch")
        self.assertEqual(draft, "")

    def test_resolve_voice_engine_prefers_available_whisper_cli_in_auto(self) -> None:
        with tempfile.NamedTemporaryFile() as model:
            config = VoiceInputConfig(whisper_cli_model=Path(model.name))
            with mock.patch("llm_cli_chat.voice.shutil.which", return_value="/bin/whisper-cli"):
                self.assertEqual(resolve_voice_engine(config), "whisper-cli")

    def test_resolve_voice_engine_falls_back_to_python_when_cli_unavailable(self) -> None:
        config = VoiceInputConfig(whisper_cli_model=Path("missing.bin"))
        with mock.patch("llm_cli_chat.voice.shutil.which", return_value=None):
            self.assertEqual(resolve_voice_engine(config), "python")


if __name__ == "__main__":
    unittest.main()
