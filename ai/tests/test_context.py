from __future__ import annotations

import asyncio
import json
import unittest
from unittest.mock import patch

from llm_cli_chat.context import (
    HOLOLENS_COMMAND_EVENT_TYPE,
    HOLOLENS_COMMAND_SOURCE,
    MissionContextProvider,
    compact_biometrics,
    compact_full_procedures,
    compact_procedures,
    format_prompt_with_context,
    normalize_hololens_window,
    normalize_user_eva,
)


class MissionContextTests(unittest.TestCase):
    def test_compact_biometrics_keeps_current_eva_metrics(self) -> None:
        payload = {
            "telemetry": {
                "eva1": {
                    "heart_rate": 82,
                    "temperature": 21.4,
                    "unrelated": "ignored",
                },
                "time": 123,
            },
            "status": {"started": True},
        }

        self.assertEqual(
            compact_biometrics(payload),
            {
                "eva1": {
                    "heart_rate": 82,
                    "temperature": 21.4,
                },
                "time": 123,
            },
        )

    def test_compact_procedures_keeps_only_names(self) -> None:
        payload = {
            "Cable Repair": {
                "name": "Cable Repair",
                "category": "Built-In",
                "description": "Procedure to Perform Cable Repair at Worksite",
                "duration": "5 mins",
                "tasks": [
                    {
                        "name": "COMM Tower Screen",
                        "description": "Perform steps at COMM Tower Screen",
                        "steps": [
                            {"type": "text", "body": "1. EV1 Select Gear icon"},
                            {"type": "image", "data": "a" * 1000},
                            {
                                "type": "text",
                                "body": "Continue with the next procedure",
                                "nextTask": [{"procedure": "Other", "task": 2}],
                            },
                        ],
                    }
                ],
            }
        }

        self.assertEqual(compact_procedures(payload), ["Cable Repair"])

    def test_compact_full_procedures_keeps_text_step_bodies(self) -> None:
        payload = {
            "Cable Repair": {
                "name": "Cable Repair",
                "category": "Built-In",
                "description": "Procedure to Perform Cable Repair at Worksite",
                "duration": "5 mins",
                "tasks": [
                    {
                        "name": "COMM Tower Screen",
                        "description": "Perform steps at COMM Tower Screen",
                        "steps": [
                            {"type": "text", "body": "1. EV1 Select Gear icon"},
                            {"type": "image", "data": "a" * 1000},
                            {
                                "type": "text",
                                "body": "Continue with the next procedure",
                                "nextTask": [{"procedure": "Other", "task": 2}],
                            },
                        ],
                    }
                ],
            }
        }

        self.assertEqual(
            compact_full_procedures(payload),
            [
                {
                    "name": "Cable Repair",
                    "category": "Built-In",
                    "description": "Procedure to Perform Cable Repair at Worksite",
                    "duration": "5 mins",
                    "tasks": [
                        {
                            "name": "COMM Tower Screen",
                            "description": "Perform steps at COMM Tower Screen",
                            "steps": [
                                {"type": "text", "body": "1. EV1 Select Gear icon"},
                                {"type": "image", "body": "[image data omitted]"},
                                {
                                    "type": "text",
                                    "body": "Continue with the next procedure",
                                    "nextTask": [{"procedure": "Other", "task": 2}],
                                },
                            ],
                        }
                    ],
                }
            ],
        )

    def test_format_prompt_places_context_before_user_prompt(self) -> None:
        context = {"available_procedures": {"status": "ok", "data": []}}

        message = format_prompt_with_context("What should I do?", context)

        self.assertLess(message.index("<ground_control_context>"), message.index("<user_prompt>"))
        self.assertIn(json.dumps(context, indent=2, sort_keys=True), message)
        self.assertIn("Call get_current_biometrics", message)
        self.assertIn("Call get_procedure", message)
        self.assertIn("do not infer procedure details from names", message)
        self.assertIn("Do not call a tool when", message)
        self.assertIn("audio transcripts", message)
        self.assertIn("similar-sounding", message)
        self.assertIn("the user is that EVA astronaut", message)
        self.assertIn("mention only that user's EVA", message)
        self.assertIn("say you do not know", message)
        self.assertIn("Hololens command tools", message)
        self.assertTrue(message.rstrip().endswith("</user_prompt>"))

    def test_biometrics_tool_text_filters_selected_eva(self) -> None:
        provider = MissionContextProvider(
            tss_endpoint="http://tss",
            timeout_seconds=1,
            user_eva="eva1",
        )
        payload = {
            "telemetry": {
                "eva1": {"heart_rate": 82, "temperature": 21.4},
                "eva2": {"heart_rate": 91},
                "time": 123,
            },
        }

        with patch("llm_cli_chat.context._fetch_json", return_value=payload):
            result = asyncio.run(provider.fetch_current_biometrics_text())

        self.assertIn('"status": "ok"', result)
        self.assertIn('"eva1"', result)
        self.assertIn('"heart_rate": 82', result)
        self.assertNotIn('"eva2"', result)

    def test_procedure_tool_text_fetches_full_matching_procedure(self) -> None:
        provider = MissionContextProvider(
            ground_control_api_url="http://ground",
            timeout_seconds=1,
        )
        payload = {
            "Cable Repair": {
                "name": "Cable Repair",
                "tasks": [
                    {
                        "name": "COMM Tower Screen",
                        "steps": [{"type": "text", "body": "1. EV1 Select Gear icon"}],
                    }
                ],
            },
        }

        with patch("llm_cli_chat.context._fetch_json", return_value=payload):
            result = asyncio.run(provider.fetch_procedure_text("cable"))

        self.assertIn('"status": "ok"', result)
        self.assertIn('"name": "Cable Repair"', result)
        self.assertIn("1. EV1 Select Gear icon", result)

    def test_normalize_user_eva_accepts_common_flag_values(self) -> None:
        self.assertEqual(normalize_user_eva("1"), "eva1")
        self.assertEqual(normalize_user_eva("EVA 2"), "eva2")
        self.assertIsNone(normalize_user_eva("eva3"))

    def test_normalize_hololens_window_accepts_aliases(self) -> None:
        self.assertEqual(normalize_hololens_window("map"), "navigation")
        self.assertEqual(normalize_hololens_window("EVA Summary Timeline"), "summary_timeline")
        self.assertIsNone(normalize_hololens_window("unknown"))

    def test_open_hololens_window_posts_command_event(self) -> None:
        provider = MissionContextProvider(
            ground_control_api_url="http://ground",
            timeout_seconds=1,
        )
        api_response = {
            "message": "Hololens command sent",
            "event": {"type": HOLOLENS_COMMAND_EVENT_TYPE},
        }

        with patch("llm_cli_chat.context._post_json", return_value=api_response) as post_json:
            result = asyncio.run(provider.open_hololens_window("map"))

        post_json.assert_called_once()
        url, command, timeout = post_json.call_args.args
        self.assertEqual(url, "http://ground/hololens/commands")
        self.assertEqual(timeout, 1)
        self.assertEqual(
            command,
            {
                "action": "open_window",
                "window": "navigation",
                "target": "hololens",
                "source": HOLOLENS_COMMAND_SOURCE,
            },
        )

        payload = json.loads(result)
        self.assertEqual(payload["status"], "sent")
        self.assertEqual(payload["event_type"], HOLOLENS_COMMAND_EVENT_TYPE)
        self.assertEqual(payload["command"], command)
        self.assertEqual(payload["response"], api_response)

    def test_close_hololens_window_rejects_unknown_window(self) -> None:
        provider = MissionContextProvider(
            ground_control_api_url="http://ground",
            timeout_seconds=1,
        )

        with patch("llm_cli_chat.context._post_json") as post_json:
            result = asyncio.run(provider.close_hololens_window("unknown"))

        post_json.assert_not_called()
        payload = json.loads(result)
        self.assertEqual(payload["status"], "invalid_window")
        self.assertEqual(payload["event_type"], HOLOLENS_COMMAND_EVENT_TYPE)

    def test_close_hololens_window_posts_command_event(self) -> None:
        provider = MissionContextProvider(
            ground_control_api_url="http://ground",
            timeout_seconds=1,
        )

        with patch("llm_cli_chat.context._post_json", return_value={}) as post_json:
            result = asyncio.run(provider.close_hololens_window("settings"))

        post_json.assert_called_once()
        _, command, _ = post_json.call_args.args
        self.assertEqual(
            command,
            {
                "action": "close_window",
                "window": "settings",
                "target": "hololens",
                "source": HOLOLENS_COMMAND_SOURCE,
            },
        )

        payload = json.loads(result)
        self.assertEqual(payload["status"], "sent")
        self.assertEqual(payload["event_type"], HOLOLENS_COMMAND_EVENT_TYPE)

    def test_open_hololens_procedure_posts_command_event(self) -> None:
        provider = MissionContextProvider(
            ground_control_api_url="http://ground",
            timeout_seconds=1,
        )

        with patch("llm_cli_chat.context._post_json", return_value={}) as post_json:
            result = asyncio.run(provider.open_hololens_procedure("Cable Repair"))

        post_json.assert_called_once()
        _, command, _ = post_json.call_args.args
        self.assertEqual(
            command,
            {
                "action": "open_procedure",
                "procedure": "Cable Repair",
                "window": "procedures",
                "target": "hololens",
                "source": HOLOLENS_COMMAND_SOURCE,
            },
        )

        payload = json.loads(result)
        self.assertEqual(payload["status"], "sent")
        self.assertEqual(payload["event_type"], HOLOLENS_COMMAND_EVENT_TYPE)


if __name__ == "__main__":
    unittest.main()
