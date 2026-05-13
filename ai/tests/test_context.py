from __future__ import annotations

import json
import unittest

from llm_cli_chat.context import (
    compact_biometrics,
    compact_procedures,
    format_prompt_with_context,
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

    def test_compact_procedures_keeps_text_step_bodies(self) -> None:
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
            compact_procedures(payload),
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
        self.assertIn("only authoritative source", message)
        self.assertIn("not by adding your own operational guidance", message)
        self.assertIn("say you do not know", message)
        self.assertTrue(message.rstrip().endswith("</user_prompt>"))

    def test_normalize_user_eva_accepts_common_flag_values(self) -> None:
        self.assertEqual(normalize_user_eva("1"), "eva1")
        self.assertEqual(normalize_user_eva("EVA 2"), "eva2")
        self.assertIsNone(normalize_user_eva("eva3"))


if __name__ == "__main__":
    unittest.main()
