from __future__ import annotations

import unittest

from llm_cli_chat.agent import DEFAULT_INSTRUCTION, DEFAULT_MODEL


class AgentConfigTests(unittest.TestCase):
    def test_default_model_uses_llama_cpp_generic_provider(self) -> None:
        self.assertTrue(DEFAULT_MODEL.startswith("generic."))
        self.assertIn("Qwen3-Coder-30B-A3B-Instruct-Q8_0-GGUF", DEFAULT_MODEL)

    def test_default_instruction_is_tts_concise(self) -> None:
        self.assertIn("few words", DEFAULT_INSTRUCTION)
        self.assertIn("TTS", DEFAULT_INSTRUCTION)
        self.assertIn("use only the provided biometrics", DEFAULT_INSTRUCTION)
        self.assertIn("available procedures", DEFAULT_INSTRUCTION)
        self.assertIn("say you do not know", DEFAULT_INSTRUCTION)


if __name__ == "__main__":
    unittest.main()
