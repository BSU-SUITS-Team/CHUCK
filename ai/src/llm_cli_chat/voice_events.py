from __future__ import annotations

import json
from collections.abc import AsyncIterator
from typing import Any
from urllib.parse import urlparse, urlunparse


VOICE_TRANSCRIPTION_EVENT_TYPE = "voice_transcription_command"
VOICE_TRANSCRIPTION_ACTIONS = {"toggle", "start", "stop"}


def events_websocket_url(ground_control_api_url: str) -> str:
    api_url = ground_control_api_url.strip()
    if "://" not in api_url:
        api_url = f"http://{api_url}"

    parsed = urlparse(api_url)
    scheme = "wss" if parsed.scheme in {"https", "wss"} else "ws"
    path = f"{parsed.path.rstrip('/')}/ws/events" if parsed.path.rstrip("/") else "/ws/events"
    return urlunparse((scheme, parsed.netloc, path, "", "", ""))


def event_time_ns(event: dict[str, Any]) -> int | None:
    try:
        event_time = int(event.get("time"))
    except (TypeError, ValueError):
        return None

    return event_time if event_time >= 0 else None


def voice_transcription_command_action(
    event: dict[str, Any],
    *,
    ignore_before_ns: int = 0,
) -> str | None:
    if event.get("type") != VOICE_TRANSCRIPTION_EVENT_TYPE:
        return None

    event_time = event_time_ns(event)
    if event_time is None or event_time < ignore_before_ns:
        return None

    data = event.get("data")
    if not isinstance(data, dict):
        return None

    action = str(data.get("action") or "").casefold()
    if action not in VOICE_TRANSCRIPTION_ACTIONS:
        return None

    return action


async def stream_voice_transcription_commands(
    websocket_url: str,
    *,
    ignore_before_ns: int = 0,
) -> AsyncIterator[tuple[str, dict[str, Any]]]:
    import websockets

    async with websockets.connect(websocket_url) as websocket:
        async for message in websocket:
            event = parse_websocket_event(message)
            if event is None:
                continue

            action = voice_transcription_command_action(
                event,
                ignore_before_ns=ignore_before_ns,
            )
            if action is not None:
                yield action, event


def parse_websocket_event(message: str | bytes) -> dict[str, Any] | None:
    if isinstance(message, bytes):
        message = message.decode("utf-8")

    try:
        event = json.loads(message)
    except json.JSONDecodeError:
        return None

    return event if isinstance(event, dict) else None
