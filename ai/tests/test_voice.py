from __future__ import annotations

import tempfile
import unittest
from pathlib import Path
from unittest import mock

from llm_cli_chat.voice import (
    VoiceInputConfig,
    build_whisper_stream_command,
    clean_whisper_stream_text,
    current_whisper_stream_transcript,
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
