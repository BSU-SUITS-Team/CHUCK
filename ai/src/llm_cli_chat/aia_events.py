from __future__ import annotations

import asyncio
from typing import Any

from llm_cli_chat.context import _join_url, _post_json


AIA_MESSAGE_EVENT_TYPE = "aia_message"
DEFAULT_AIA_MESSAGE_TIMEOUT_SECONDS = 2.0


async def send_aia_message_event(
    ground_control_api_url: str,
    message: str,
    *,
    timeout_seconds: float = DEFAULT_AIA_MESSAGE_TIMEOUT_SECONDS,
    source: str = "aia",
    target: str = "hololens",
) -> dict[str, Any] | None:
    cleaned_message = message.strip()
    if not cleaned_message:
        return None

    url = _join_url(ground_control_api_url, "/aia/messages")
    payload = {
        "message": cleaned_message,
        "source": source,
        "target": target,
    }
    return await asyncio.to_thread(_post_json, url, payload, timeout_seconds)


class AiaMessageLineSender:
    def __init__(
        self,
        ground_control_api_url: str,
        *,
        timeout_seconds: float = DEFAULT_AIA_MESSAGE_TIMEOUT_SECONDS,
        source: str = "aia",
        target: str = "hololens",
    ) -> None:
        self.ground_control_api_url = ground_control_api_url
        self.timeout_seconds = timeout_seconds
        self.source = source
        self.target = target
        self._cursor = 0

    async def send_completed_lines(self, accumulated_message: str) -> int:
        if self._cursor > len(accumulated_message):
            self._cursor = 0

        sent_count = 0
        while True:
            newline_index = accumulated_message.find("\n", self._cursor)
            if newline_index == -1:
                return sent_count

            line_start = self._cursor
            next_cursor = newline_index + 1
            line = accumulated_message[line_start:newline_index].strip()
            if not line:
                self._cursor = next_cursor
                continue

            await self._send(line)
            self._cursor = next_cursor
            sent_count += 1

    async def send_remaining(self, accumulated_message: str) -> bool:
        if self._cursor > len(accumulated_message):
            self._cursor = 0

        remaining_message = accumulated_message[self._cursor :].strip()
        if not remaining_message:
            self._cursor = len(accumulated_message)
            return False

        await self._send(remaining_message)
        self._cursor = len(accumulated_message)
        return True

    async def _send(self, message: str) -> None:
        await send_aia_message_event(
            self.ground_control_api_url,
            message,
            timeout_seconds=self.timeout_seconds,
            source=self.source,
            target=self.target,
        )
