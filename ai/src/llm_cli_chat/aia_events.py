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
