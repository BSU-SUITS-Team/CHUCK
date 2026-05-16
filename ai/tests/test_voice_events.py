from __future__ import annotations

import json
import unittest

from llm_cli_chat.voice_events import (
    event_time_ns,
    events_websocket_url,
    parse_websocket_event,
    voice_transcription_command_action,
)


class VoiceEventTests(unittest.TestCase):
    def test_events_websocket_url_uses_ground_control_events_path(self) -> None:
        self.assertEqual(
            events_websocket_url("http://localhost:8181"),
            "ws://localhost:8181/ws/events",
        )
        self.assertEqual(
            events_websocket_url("https://ground.example/api"),
            "wss://ground.example/api/ws/events",
        )

    def test_voice_transcription_command_action_accepts_current_toggle_events(self) -> None:
        event = {
            "type": "voice_transcription_command",
            "time": 20,
            "data": {"action": "toggle"},
        }

        self.assertEqual(
            voice_transcription_command_action(event, ignore_before_ns=10),
            "toggle",
        )

    def test_voice_transcription_command_action_ignores_replayed_old_events(self) -> None:
        event = {
            "type": "voice_transcription_command",
            "time": 9,
            "data": {"action": "toggle"},
        }

        self.assertIsNone(voice_transcription_command_action(event, ignore_before_ns=10))

    def test_voice_transcription_command_action_rejects_unrelated_events(self) -> None:
        self.assertIsNone(
            voice_transcription_command_action(
                {
                    "type": "hololens_command",
                    "time": 20,
                    "data": {"action": "toggle"},
                },
            )
        )
        self.assertIsNone(
            voice_transcription_command_action(
                {
                    "type": "voice_transcription_command",
                    "time": 20,
                    "data": {"action": "open_window"},
                },
            )
        )

    def test_parse_websocket_event_accepts_text_and_bytes(self) -> None:
        event = {"type": "voice_transcription_command", "time": 1, "data": {}}

        self.assertEqual(parse_websocket_event(json.dumps(event)), event)
        self.assertEqual(parse_websocket_event(json.dumps(event).encode("utf-8")), event)
        self.assertIsNone(parse_websocket_event("not json"))

    def test_event_time_ns_returns_none_for_invalid_event_times(self) -> None:
        self.assertEqual(event_time_ns({"time": "42"}), 42)
        self.assertIsNone(event_time_ns({"time": -1}))
        self.assertIsNone(event_time_ns({}))


if __name__ == "__main__":
    unittest.main()
