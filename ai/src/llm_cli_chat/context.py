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
    return compact_procedure_names(raw_procedures)


def compact_procedure_names(raw_procedures: Any) -> list[str]:
    if isinstance(raw_procedures, dict):
        procedures = raw_procedures.items()
    elif isinstance(raw_procedures, list):
        procedures = enumerate(raw_procedures)
    else:
        return []

    names: set[str] = set()
    for fallback_name, procedure in procedures:
        name = _procedure_name(fallback_name, procedure)
        if name:
            names.add(name)

    return sorted(names, key=str.casefold)


def compact_full_procedures(raw_procedures: Any) -> list[dict[str, Any]]:
    if isinstance(raw_procedures, dict):
        procedures = raw_procedures.items()
    elif isinstance(raw_procedures, list):
        procedures = enumerate(raw_procedures)
    else:
        return []

    compacted: list[dict[str, Any]] = []
    for fallback_name, procedure in procedures:
        compacted_procedure = _compact_procedure(fallback_name, procedure)
        if compacted_procedure:
            compacted.append(compacted_procedure)

    return sorted(compacted, key=lambda procedure: procedure["name"].casefold())


def _procedure_name(fallback_name: Any, procedure: Any) -> str:
    if isinstance(procedure, dict):
        return _text(procedure.get("name") or fallback_name, max_length=120)
    return _text(procedure or fallback_name, max_length=120)


def _compact_procedure(fallback_name: Any, procedure: Any) -> dict[str, Any]:
    if not isinstance(procedure, dict):
        name = _text(procedure or fallback_name, max_length=120)
        return {"name": name} if name else {}

    compacted: dict[str, Any] = {}
    name = _text(procedure.get("name") or fallback_name, max_length=120)
    if not name:
        return {}
    compacted["name"] = name

    for field in ("category", "description", "duration"):
        value = _text(procedure.get(field))
        if value:
            compacted[field] = value

    tasks = _compact_tasks(procedure.get("tasks"))
    if tasks:
        compacted["tasks"] = tasks

    return compacted


def _compact_tasks(raw_tasks: Any) -> list[dict[str, Any]]:
    if not isinstance(raw_tasks, list):
        return []

    tasks: list[dict[str, Any]] = []
    for task in raw_tasks:
        compacted_task = _compact_task(task)
        if compacted_task:
            tasks.append(compacted_task)

    return tasks


def _compact_task(task: Any) -> dict[str, Any]:
    if not isinstance(task, dict):
        name = _text(task, max_length=120)
        return {"name": name} if name else {}

    compacted: dict[str, Any] = {}
    name = _text(task.get("name"), max_length=120)
    if name:
        compacted["name"] = name

    description = _text(task.get("description"))
    if description:
        compacted["description"] = description

    steps = _compact_steps(task.get("steps"))
    if steps:
        compacted["steps"] = steps

    return compacted


def _compact_steps(raw_steps: Any) -> list[dict[str, Any]]:
    if not isinstance(raw_steps, list):
        return []

    steps: list[dict[str, Any]] = []
    for step in raw_steps:
        compacted_step = _compact_step(step)
        if compacted_step:
            steps.append(compacted_step)

    return steps


def _compact_step(step: Any) -> dict[str, Any]:
    if not isinstance(step, dict):
        body = _text(step, max_length=1_000)
        return {"type": "text", "body": body} if body else {}

    compacted: dict[str, Any] = {}
    step_type = _text(step.get("type"), max_length=60)
    if step_type:
        compacted["type"] = step_type

    is_image = step_type.casefold() == "image"
    body = _text(step.get("body"), max_length=1_000)
    if is_image and ("body" in step or "data" in step):
        compacted["body"] = "[image data omitted]"
    elif body:
        compacted["body"] = body

    next_task = _compact_next_task(step.get("nextTask"))
    if next_task is not None:
        compacted["nextTask"] = next_task

    return compacted


def _compact_next_task(next_task: Any) -> Any | None:
    if not next_task:
        return None

    if not isinstance(next_task, (dict, list, str, int, float, bool)):
        return None

    try:
        serialized = json.dumps(next_task, sort_keys=True)
    except TypeError:
        return None

    if len(serialized) > 500:
        return None

    return next_task


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
        procedures_result = await self._fetch_procedures()
        context = {
            "available_procedures": procedures_result.to_dict(),
            "mission_data_tools": {
                "current_biometrics": (
                    "Call get_current_biometrics for current biometric values."
                ),
                "procedure_details": (
                    "Call get_procedure for one full procedure or get_all_procedures "
                    "to search all procedure details."
                ),
            },
        }
        if self.user_eva is not None:
            context["user"] = {
                "eva": self.user_eva,
                "label": self.user_eva.upper().replace("EVA", "EVA "),
            }

        return context

    async def fetch_current_biometrics_text(self, eva: str | None = None) -> str:
        source = _join_url(self.tss_endpoint, "/data/EVA.json")
        selected_eva = normalize_user_eva(eva) or self.user_eva
        try:
            payload = await asyncio.to_thread(_fetch_json, source, self.timeout_seconds)
        except (HTTPError, URLError, TimeoutError, OSError, json.JSONDecodeError) as exc:
            return _tool_json(
                {
                    "source": source,
                    "status": "unavailable",
                    "error": str(exc),
                }
            )

        biometrics = compact_biometrics(payload)
        data: dict[str, Any] = biometrics
        if selected_eva is not None:
            data = {}
            if selected_eva in biometrics:
                data[selected_eva] = biometrics[selected_eva]
            if "time" in biometrics:
                data["time"] = biometrics["time"]

        return _tool_json(
            {
                "source": source,
                "status": "ok",
                "selected_eva": selected_eva,
                "data": data,
            }
        )

    async def fetch_procedure_text(self, procedure_name: str) -> str:
        source = _join_url(self.ground_control_api_url, "/procedures/")
        try:
            payload = await asyncio.to_thread(_fetch_json, source, self.timeout_seconds)
        except (HTTPError, URLError, TimeoutError, OSError, json.JSONDecodeError) as exc:
            return _tool_json(
                {
                    "source": source,
                    "status": "unavailable",
                    "error": str(exc),
                }
            )

        procedures = compact_full_procedures(payload)
        query = procedure_name.strip().casefold()
        exact_match = next(
            (
                procedure
                for procedure in procedures
                if procedure.get("name", "").casefold() == query
            ),
            None,
        )
        if exact_match is not None:
            return _tool_json(
                {
                    "source": source,
                    "status": "ok",
                    "data": exact_match,
                }
            )

        partial_matches = [
            procedure
            for procedure in procedures
            if query and query in procedure.get("name", "").casefold()
        ]
        if len(partial_matches) == 1:
            return _tool_json(
                {
                    "source": source,
                    "status": "ok",
                    "data": partial_matches[0],
                }
            )

        return _tool_json(
            {
                "source": source,
                "status": "not_found",
                "requested_procedure": procedure_name,
                "matching_procedure_names": [
                    procedure["name"] for procedure in partial_matches
                ],
                "available_procedure_names": [
                    procedure["name"] for procedure in procedures
                ],
            }
        )

    async def fetch_all_procedures_text(self) -> str:
        source = _join_url(self.ground_control_api_url, "/procedures/")
        try:
            payload = await asyncio.to_thread(_fetch_json, source, self.timeout_seconds)
        except (HTTPError, URLError, TimeoutError, OSError, json.JSONDecodeError) as exc:
            return _tool_json(
                {
                    "source": source,
                    "status": "unavailable",
                    "error": str(exc),
                }
            )

        return _tool_json(
            {
                "source": source,
                "status": "ok",
                "data": compact_full_procedures(payload),
            }
        )

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
        "biometrics. For EVA mission-state and procedure questions, the fetched "
        "context and available mission-data tools are the only authoritative "
        "sources. The context only includes procedure names; do not infer "
        "procedure details from names. Call get_current_biometrics before "
        "answering questions about current biometric values. Call get_procedure "
        "before answering questions about a specific procedure's steps, tasks, "
        "or details. Call get_all_procedures when a question asks which "
        "procedures contain some detail or needs comparison across procedures. "
        "Report information from those tools instead of adding your own "
        "operational guidance. If requested information is not present in the "
        "tool data or a needed source is unavailable, say you do not know.\n\n"
        "<ground_control_context>\n"
        f"{context_json}\n"
        "</ground_control_context>\n\n"
        "<user_prompt>\n"
        f"{user_prompt}\n"
        "</user_prompt>"
    )


def _tool_json(payload: dict[str, Any]) -> str:
    return json.dumps(payload, indent=2, sort_keys=True)
