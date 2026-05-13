from __future__ import annotations

import unittest

from llm_cli_chat.agent import DEFAULT_INSTRUCTION, DEFAULT_MODEL


class AgentConfigTests(unittest.TestCase):
    def test_default_model_uses_low_reasoning(self) -> None:
        self.assertIn("reasoning=low", DEFAULT_MODEL)

    def test_default_instruction_is_tts_concise(self) -> None:
        self.assertIn("few words", DEFAULT_INSTRUCTION)
        self.assertIn("TTS", DEFAULT_INSTRUCTION)


if __name__ == "__main__":
    unittest.main()
