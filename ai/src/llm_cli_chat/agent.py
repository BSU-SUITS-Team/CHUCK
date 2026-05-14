from __future__ import annotations

import asyncio
from collections.abc import AsyncIterator
from copy import deepcopy
from pathlib import Path
from types import TracebackType
from typing import Any

from fast_agent import FastAgent
from fast_agent.types import RequestParams

from llm_cli_chat.context import MissionContextProvider


DEFAULT_AGENT_NAME = "assistant"
DEFAULT_MODEL = "generic.qwen3.6:35b-a3b?reasoning=off"
TOOL_STATUS_MESSAGE = "Checking current data...\n\n"
OLLAMA_NO_THINK_METADATA = {
    "reasoning_effort": "none",
    "extra_body": {
        "think": False,
    },
}
DEFAULT_INSTRUCTION = (
    "You are a concise, helpful assistant for an EVA terminal chat application. "
    "Respond with only a few words whenever possible. Responses are spoken "
    "through TTS and are time sensitive. For EVA mission-state and procedure "
    "questions, use only the provided context and mission-data tools. Procedure "
    "names in context are an index, not enough detail to answer procedural "
    "questions. Call get_current_biometrics before reporting current biometric "
    "values. Call get_procedure before reporting a specific procedure's steps "
    "or details. Call get_all_procedures when the question asks which "
    "procedures contain some detail or needs comparison across procedures. "
    "Provide information from tool results instead of giving your own "
    "operational guidance. If the answer is not in the tool results or a "
    "listed procedure name, say you do not know. Ask clarifying questions only "
    "when needed for safety or correctness."
)


def build_fast_agent(
    *,
    model: str | None = DEFAULT_MODEL,
    config_path: Path | None = None,
    agent_name: str = DEFAULT_AGENT_NAME,
    instruction: str = DEFAULT_INSTRUCTION,
    context_provider: MissionContextProvider | None = None,
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

    request_params = no_thinking_request_params(model)
    if request_params is not None:
        agent_kwargs["request_params"] = request_params

    @fast.agent(**agent_kwargs)
    async def chat_agent() -> None:
        pass

    if context_provider is not None:

        @chat_agent.tool(
            name="get_current_biometrics",
            description=(
                "Fetch current EVA biometrics from TSS. Use before reporting "
                "heart rate, temperature, suit pressure, oxygen, CO2, battery, "
                "or other current biometric/telemetry values."
            ),
        )
        async def get_current_biometrics(eva: str | None = None) -> str:
            return await context_provider.fetch_current_biometrics_text(eva)

        @chat_agent.tool(
            name="get_procedure",
            description=(
                "Fetch the full text, tasks, and steps for one available "
                "procedure by name."
            ),
        )
        async def get_procedure(procedure_name: str) -> str:
            return await context_provider.fetch_procedure_text(procedure_name)

        @chat_agent.tool(
            name="get_all_procedures",
            description=(
                "Fetch full text, tasks, and steps for all available "
                "procedures. Use when searching procedure details across "
                "multiple procedures."
            ),
        )
        async def get_all_procedures() -> str:
            return await context_provider.fetch_all_procedures_text()

    return fast


def no_thinking_request_params(model: str | None) -> RequestParams | None:
    if not model:
        return None

    normalized = model.partition("?")[0].casefold()
    if normalized.startswith("generic.qwen3.6"):
        return RequestParams(metadata=deepcopy(OLLAMA_NO_THINK_METADATA))
    return None


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
            context_provider=self.context_provider,
        )
        self._run_context = self._fast.run()
        self._agents = await self._run_context.__aenter__()

    async def send(self, message: str) -> str:
        if not self.is_started:
            await self.start()

        message = await self._build_contextual_message(message)
        agent = self._agents.get_agent(self.agent_name)
        tool_notice_sent = False

        def on_tool_event(event_type: str, info: dict[str, Any] | None = None) -> None:
            nonlocal tool_notice_sent
            if event_type == "start":
                tool_notice_sent = True

        remove_tool_listener = None
        if agent is not None and hasattr(agent, "add_tool_stream_listener"):
            remove_tool_listener = agent.add_tool_stream_listener(on_tool_event)

        try:
            result = await self._agents.send(message, agent_name=self.agent_name)
        finally:
            if remove_tool_listener is not None:
                remove_tool_listener()

        response = str(result)
        if tool_notice_sent:
            return f"{TOOL_STATUS_MESSAGE}{response}"
        return response

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
        tool_notice_sent = False

        def on_tool_event(event_type: str, info: dict[str, Any] | None = None) -> None:
            nonlocal tool_notice_sent
            if event_type != "start" or tool_notice_sent:
                return
            tool_notice_sent = True
            loop.call_soon_threadsafe(queue.put_nowait, TOOL_STATUS_MESSAGE)

        remove_tool_listener = None
        if hasattr(agent, "add_tool_stream_listener"):
            remove_tool_listener = agent.add_tool_stream_listener(on_tool_event)
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
            if remove_tool_listener is not None:
                remove_tool_listener()

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
