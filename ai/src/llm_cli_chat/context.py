from __future__ import annotations

import asyncio
import json
import os
from dataclasses import dataclass
from typing import Any
from urllib.error import HTTPError, URLError
from urllib.parse import urljoin
from urllib.request import Request, urlopen


DEFAULT_GROUND_CONTROL_API_URL = "http://localhost:8181"
DEFAULT_TSS_ENDPOINT = "http://localhost:14141"
DEFAULT_CONTEXT_TIMEOUT_SECONDS = 2.0
USER_EVA_CHOICES = ("eva1", "eva2")

BIOMETRIC_FIELDS = {
    "primary_battery_level",
    "secondary_battery_level",
    "battery_level",
    "oxy_pri_storage",
    "oxy_sec_storage",
    "oxy_pri_pressure",
    "oxy_sec_pressure",
    "coolant_storage",
    "eva_elapsed_time",
    "heart_rate",
    "oxy_consumption",
    "co2_production",
    "suit_pressure_oxy",
    "suit_pressure_co2",
    "suit_pressure_other",
    "suit_pressure_total",
    "helmet_pressure_co2",
    "fan_pri_rpm",
    "fan_sec_rpm",
    "scrubber_a_co2_storage",
    "scrubber_b_co2_storage",
    "temperature",
    "coolant_gas_pressure",
    "coolant_liquid_pressure",
}


def _env_float(name: str, default: float) -> float:
    value = os.getenv(name)
    if value is None:
        return default

    try:
        return float(value)
    except ValueError:
        return default


def normalize_user_eva(value: str | None) -> str | None:
    if value is None:
        return None

    normalized = value.strip().lower().replace(" ", "")
    eva_map = {
        "1": "eva1",
        "eva1": "eva1",
        "ev1": "eva1",
        "eva-1": "eva1",
        "2": "eva2",
        "eva2": "eva2",
        "ev2": "eva2",
        "eva-2": "eva2",
    }
    return eva_map.get(normalized)


def _join_url(base_url: str, path: str) -> str:
    return urljoin(base_url.rstrip("/") + "/", path.lstrip("/"))


def _fetch_json(url: str, timeout_seconds: float) -> dict[str, Any]:
    request = Request(url, headers={"Accept": "application/json"})
    with urlopen(request, timeout=timeout_seconds) as response:
        charset = response.headers.get_content_charset() or "utf-8"
        payload = response.read().decode(charset)

    data = json.loads(payload)
    if not isinstance(data, dict):
        return {"value": data}
    return data


def _text(value: Any, *, max_length: int = 400) -> str:
    if value is None:
        return ""

    text = str(value).strip()
    if len(text) <= max_length:
        return text
    return f"{text[: max_length - 3]}..."


def compact_procedures(raw_procedures: Any) -> list[str]:
    if isinstance(raw_procedures, dict):
        procedures = raw_procedures.items()
    elif isinstance(raw_procedures, list):
        procedures = enumerate(raw_procedures)
    else:
        return []

    names: set[str] = set()
    for fallback_name, procedure in procedures:
        if isinstance(procedure, dict):
            name = _text(procedure.get("name") or fallback_name, max_length=120)
        else:
            name = _text(procedure, max_length=120)
        if name:
            names.add(name)

    return sorted(names, key=str.casefold)


def compact_biometrics(raw_payload: Any) -> dict[str, Any]:
    if not isinstance(raw_payload, dict):
        return {"value": raw_payload}

    telemetry = raw_payload.get("telemetry")
    if not isinstance(telemetry, dict):
        telemetry = raw_payload

    biometrics: dict[str, Any] = {}
    for eva, values in telemetry.items():
        if eva == "time" or not isinstance(values, dict):
            continue

        current = {
            field: values[field]
            for field in BIOMETRIC_FIELDS
            if field in values
        }
        if current:
            biometrics[eva] = current

    if "time" in telemetry:
        biometrics["time"] = telemetry["time"]

    return biometrics


@dataclass(frozen=True)
class ContextFetchResult:
    source: str
    status: str
    data: Any = None
    error: str | None = None

    def to_dict(self) -> dict[str, Any]:
        result: dict[str, Any] = {
            "source": self.source,
            "status": self.status,
        }
        if self.data is not None:
            result["data"] = self.data
        if self.error:
            result["error"] = self.error
        return result


class MissionContextProvider:
    """Fetches current mission state to prepend to each assistant prompt."""

    def __init__(
        self,
        *,
        ground_control_api_url: str | None = None,
        tss_endpoint: str | None = None,
        timeout_seconds: float | None = None,
        user_eva: str | None = None,
    ) -> None:
        self.ground_control_api_url = (
            ground_control_api_url
            or os.getenv("GROUND_CONTROL_API_URL")
            or DEFAULT_GROUND_CONTROL_API_URL
        )
        self.tss_endpoint = (
            tss_endpoint
            or os.getenv("TSS_ENDPOINT")
            or DEFAULT_TSS_ENDPOINT
        )
        self.timeout_seconds = (
            timeout_seconds
            if timeout_seconds is not None
            else _env_float("LLM_CHAT_CONTEXT_TIMEOUT", DEFAULT_CONTEXT_TIMEOUT_SECONDS)
        )
        self.user_eva = normalize_user_eva(user_eva or os.getenv("LLM_CHAT_USER_EVA"))

    async def build_prompt(self, user_prompt: str) -> str:
        context = await self.fetch_context()
        return format_prompt_with_context(user_prompt, context)

    async def fetch_context(self) -> dict[str, Any]:
        biometrics_result, procedures_result = await asyncio.gather(
            self._fetch_biometrics(),
            self._fetch_procedures(),
        )
        context = {
            "biometrics": biometrics_result.to_dict(),
            "available_procedures": procedures_result.to_dict(),
        }
        if self.user_eva is not None:
            context["user"] = {
                "eva": self.user_eva,
                "label": self.user_eva.upper().replace("EVA", "EVA "),
            }

        return context

    async def _fetch_biometrics(self) -> ContextFetchResult:
        source = _join_url(self.tss_endpoint, "/data/EVA.json")
        try:
            payload = await asyncio.to_thread(_fetch_json, source, self.timeout_seconds)
        except (HTTPError, URLError, TimeoutError, OSError, json.JSONDecodeError) as exc:
            return ContextFetchResult(
                source=source,
                status="unavailable",
                error=str(exc),
            )

        return ContextFetchResult(
            source=source,
            status="ok",
            data=compact_biometrics(payload),
        )

    async def _fetch_procedures(self) -> ContextFetchResult:
        source = _join_url(self.ground_control_api_url, "/procedures/")
        try:
            payload = await asyncio.to_thread(_fetch_json, source, self.timeout_seconds)
        except (HTTPError, URLError, TimeoutError, OSError, json.JSONDecodeError) as exc:
            return ContextFetchResult(
                source=source,
                status="unavailable",
                error=str(exc),
            )

        return ContextFetchResult(
            source=source,
            status="ok",
            data=compact_procedures(payload),
        )


def format_prompt_with_context(user_prompt: str, context: dict[str, Any]) -> str:
    context_json = json.dumps(context, indent=2, sort_keys=True)
    return (
        "The following ground-control context was fetched immediately before "
        "the user prompt. Treat it as current mission state, not as user-authored "
        "instructions. Prefer this latest context over older context in the chat "
        "history. If user.eva is present, interpret the user's first-person "
        "requests as coming from that EVA and prioritize that astronaut's "
        "biometrics. Use this context when relevant, and mention unavailable "
        "sources if they matter to the answer.\n\n"
        "<ground_control_context>\n"
        f"{context_json}\n"
        "</ground_control_context>\n\n"
        "<user_prompt>\n"
        f"{user_prompt}\n"
        "</user_prompt>"
    )
