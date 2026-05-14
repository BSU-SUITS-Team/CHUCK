from __future__ import annotations

import unittest

from llm_cli_chat.agent import (
    DEFAULT_AGENT_NAME,
    DEFAULT_INSTRUCTION,
    DEFAULT_MODEL,
    OLLAMA_NO_THINK_METADATA,
    TOOL_STATUS_MESSAGE,
    build_fast_agent,
    no_thinking_request_params,
)
from llm_cli_chat.context import MissionContextProvider


class AgentConfigTests(unittest.TestCase):
    def test_default_model_uses_qwen36_ollama_generic_provider(self) -> None:
        self.assertTrue(DEFAULT_MODEL.startswith("generic."))
        self.assertIn("qwen3.6:35b-a3b", DEFAULT_MODEL)
        self.assertIn("reasoning=off", DEFAULT_MODEL)

    def test_default_instruction_is_tts_concise(self) -> None:
        self.assertIn("few words", DEFAULT_INSTRUCTION)
        self.assertIn("TTS", DEFAULT_INSTRUCTION)
        self.assertIn("mission-data tools", DEFAULT_INSTRUCTION)
        self.assertIn("Call get_current_biometrics", DEFAULT_INSTRUCTION)
        self.assertIn("Call get_procedure", DEFAULT_INSTRUCTION)
        self.assertIn("Do not call a tool when", DEFAULT_INSTRUCTION)
        self.assertIn("audio transcripts", DEFAULT_INSTRUCTION)
        self.assertIn("similar-sounding", DEFAULT_INSTRUCTION)
        self.assertIn("say you do not know", DEFAULT_INSTRUCTION)

    def test_default_model_request_params_disable_ollama_thinking(self) -> None:
        request_params = no_thinking_request_params(DEFAULT_MODEL)

        self.assertIsNotNone(request_params)
        self.assertEqual(request_params.metadata, OLLAMA_NO_THINK_METADATA)
        self.assertNotIn("reasoning", request_params.metadata)
        self.assertEqual(request_params.metadata["reasoning_effort"], "none")

    def test_build_fast_agent_registers_mission_tools(self) -> None:
        provider = MissionContextProvider()
        fast = build_fast_agent(context_provider=provider)

        config = fast.agents[DEFAULT_AGENT_NAME]["config"]
        tool_names = {
            tool_config.name
            for tool_config in config.function_tools
            if getattr(tool_config, "name", None)
        }

        self.assertEqual(
            tool_names,
            {
                "get_current_biometrics",
                "get_procedure",
                "get_all_procedures",
            },
        )
        self.assertEqual(config.default_request_params.metadata, OLLAMA_NO_THINK_METADATA)
        self.assertIn("Checking current data", TOOL_STATUS_MESSAGE)


if __name__ == "__main__":
    unittest.main()
