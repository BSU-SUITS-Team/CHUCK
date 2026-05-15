from __future__ import annotations

import tempfile
import unittest
from pathlib import Path
from unittest import mock

from llm_cli_chat.voice import (
    AudioInputDevice,
    VoiceInputConfig,
    VoiceInputController,
    audio_input_device_label,
    build_whisper_stream_command,
    clean_whisper_stream_text,
    current_whisper_stream_transcript,
    list_audio_input_devices,
    merge_transcript_update,
    resolve_voice_engine,
    update_whisper_stream_transcript,
)


class VoiceHelperTests(unittest.TestCase):
    def test_build_whisper_stream_command_matches_default_stream_settings(self) -> None:
        command = build_whisper_stream_command(
            VoiceInputConfig(whisper_stream_model=Path("ggml-base.en.bin")),
        )

        self.assertEqual(
            command,
            [
                "whisper-stream",
                "-m",
                "ggml-base.en.bin",
                "-t",
                "8",
                "--step",
                "500",
                "--length",
                "5000",
            ],
        )

    def test_build_whisper_stream_command_includes_capture_device_when_selected(self) -> None:
        command = build_whisper_stream_command(
            VoiceInputConfig(
                whisper_stream_model=Path("ggml-base.en.bin"),
                audio_input_device=3,
            ),
        )

        self.assertIn("--capture", command)
        self.assertEqual(command[command.index("--capture") + 1], "3")

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

    def test_clean_whisper_stream_text_removes_timestamps_and_status_lines(self) -> None:
        self.assertEqual(
            clean_whisper_stream_text("\x1b[32m[00:00:00.000 --> 00:00:01.000]  hello   world"),
            "hello world",
        )
        self.assertEqual(clean_whisper_stream_text("[Start speaking]  hello world"), "hello world")
        self.assertEqual(clean_whisper_stream_text("init: found 1 capture devices"), "")

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

    def test_resolve_voice_engine_prefers_available_whisper_stream_in_auto(self) -> None:
        with tempfile.NamedTemporaryFile() as model:
            config = VoiceInputConfig(whisper_stream_model=Path(model.name))
            with mock.patch("llm_cli_chat.voice.shutil.which", return_value="/bin/whisper-stream"):
                self.assertEqual(resolve_voice_engine(config), "whisper-stream")

    def test_resolve_voice_engine_falls_back_to_python_when_stream_unavailable(self) -> None:
        config = VoiceInputConfig(whisper_stream_model=Path("missing.bin"))
        with mock.patch("llm_cli_chat.voice.shutil.which", return_value=None):
            self.assertEqual(resolve_voice_engine(config), "python")


if __name__ == "__main__":
    unittest.main()
