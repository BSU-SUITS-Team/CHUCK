from __future__ import annotations

import asyncio
from collections.abc import AsyncIterator
from pathlib import Path
from types import TracebackType
from typing import Any

from fast_agent import FastAgent

from llm_cli_chat.context import MissionContextProvider


DEFAULT_AGENT_NAME = "assistant"
DEFAULT_MODEL = "generic.ggml-org/Qwen3-Coder-30B-A3B-Instruct-Q8_0-GGUF"
DEFAULT_INSTRUCTION = (
    "You are a concise, helpful assistant for an EVA terminal chat application. "
    "Respond with only a few words whenever possible. Responses are spoken "
    "through TTS and are time sensitive. For EVA mission-state and procedure "
    "questions, use only the provided biometrics and available procedures "
    "context. Provide information from procedures instead of giving your own "
    "operational guidance. If the answer is not in the biometrics or an "
    "available procedure, say you do not know. Ask clarifying questions only "
    "when needed for safety or correctness."
)


def build_fast_agent(
    *,
    model: str | None = DEFAULT_MODEL,
    config_path: Path | None = None,
    agent_name: str = DEFAULT_AGENT_NAME,
    instruction: str = DEFAULT_INSTRUCTION,
) -> FastAgent:
    """Create and register the chat agent."""
    fast_kwargs: dict[str, Any] = {
        "parse_cli_args": False,
        "quiet": True,
    }
    if config_path is not None:
        fast_kwargs["config_path"] = str(config_path)

    fast = FastAgent("LLM CLI Chat", **fast_kwargs)

    agent_kwargs: dict[str, Any] = {
        "name": agent_name,
        "instruction": instruction,
        "use_history": True,
        "default": True,
    }
    if model:
        agent_kwargs["model"] = model

    @fast.agent(**agent_kwargs)
    async def chat_agent() -> None:
        pass

    return fast


class FastAgentChatBackend:
    """Owns the fast-agent runtime for the lifetime of the TUI app."""

    def __init__(
        self,
        *,
        model: str | None = DEFAULT_MODEL,
        config_path: Path | None = None,
        agent_name: str = DEFAULT_AGENT_NAME,
        context_provider: MissionContextProvider | None = None,
        include_mission_context: bool = True,
    ) -> None:
        self.model = model
        self.config_path = config_path
        self.agent_name = agent_name
        self.context_provider = (
            context_provider
            if context_provider is not None
            else MissionContextProvider()
            if include_mission_context
            else None
        )
        self._fast: FastAgent | None = None
        self._run_context: Any | None = None
        self._agents: Any | None = None

    @property
    def is_started(self) -> bool:
        return self._agents is not None

    async def start(self) -> None:
        if self.is_started:
            return

        self._fast = build_fast_agent(
            model=self.model,
            config_path=self.config_path,
            agent_name=self.agent_name,
        )
        self._run_context = self._fast.run()
        self._agents = await self._run_context.__aenter__()

    async def send(self, message: str) -> str:
        if not self.is_started:
            await self.start()

        message = await self._build_contextual_message(message)
        result = await self._agents.send(message, agent_name=self.agent_name)
        return str(result)

    async def stream(self, message: str) -> AsyncIterator[str]:
        if not self.is_started:
            await self.start()

        agent = self._agents.get_agent(self.agent_name)
        if agent is None:
            response = await self.send(message)
            if response:
                yield response
            return

        message = await self._build_contextual_message(message)
        loop = asyncio.get_running_loop()
        queue: asyncio.Queue[str] = asyncio.Queue()
        streamed = False

        def on_chunk(chunk: Any) -> None:
            if getattr(chunk, "is_reasoning", False):
                return

            text = getattr(chunk, "text", "")
            if text:
                loop.call_soon_threadsafe(queue.put_nowait, str(text))

        remove_listener = agent.add_stream_listener(on_chunk)
        send_task = asyncio.create_task(agent.send(message))

        try:
            while True:
                if send_task.done() and queue.empty():
                    break

                try:
                    chunk = await asyncio.wait_for(queue.get(), timeout=0.05)
                except asyncio.TimeoutError:
                    continue

                streamed = True
                yield chunk

            response = await send_task
        finally:
            remove_listener()

        if not streamed and response:
            yield str(response)

    async def _build_contextual_message(self, message: str) -> str:
        if self.context_provider is None:
            return message

        return await self.context_provider.build_prompt(message)

    async def stop(
        self,
        exc_type: type[BaseException] | None = None,
        exc: BaseException | None = None,
        traceback: TracebackType | None = None,
    ) -> None:
        if self._run_context is None:
            return

        await self._run_context.__aexit__(exc_type, exc, traceback)
        self._run_context = None
        self._agents = None
        self._fast = None
