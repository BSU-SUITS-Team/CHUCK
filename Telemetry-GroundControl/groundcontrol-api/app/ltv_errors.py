import asyncio
import logging
import re
from os import getenv
from typing import Awaitable, Callable

import requests


LTV_ERRORS_URL = getenv(
    "LTV_ERRORS_ENDPOINT",
    f"{getenv('TSS_ENDPOINT', 'http://localhost:14141')}/data/LTV_ERRORS.json",
)
LTV_ERROR_POLL_INTERVAL_SECONDS = 3

_NUMBERED_STEP_BOUNDARY_RE = re.compile(r"\s+(?=\d+\.(?=\s|[^\d\s]|$))")
_NUMBERED_STEP_PREFIX_RE = re.compile(r"^\d+\.(?=\s|[^\d\s]|$)")
_MISSING_STEP_SPACE_RE = re.compile(r"^(\d+)\.(?=[^\d\s])")
_LEADING_STEP_NUMBER_RE = re.compile(r"^\d+\.(?:\s+|(?=[^\d\s])|$)")


ProcedureUpsert = Callable[[dict], Awaitable[bool]]


def _clean_step_text(step_text: str) -> str:
    normalized = re.sub(r"\s+", " ", step_text).strip()
    return _MISSING_STEP_SPACE_RE.sub(r"\1. ", normalized)


def _remove_step_number(step_text: str) -> str:
    return _LEADING_STEP_NUMBER_RE.sub("", step_text).strip()


def _filter_empty_steps(steps: list[str]) -> list[str]:
    return [step for step in steps if step.strip()]


def split_numbered_steps(step_text: str) -> list[str]:
    normalized = _clean_step_text(step_text)
    if not normalized:
        return []

    if not _NUMBERED_STEP_PREFIX_RE.match(normalized):
        return [normalized]

    return _filter_empty_steps([
        _remove_step_number(_clean_step_text(step))
        for step in _NUMBERED_STEP_BOUNDARY_RE.split(normalized)
        if step.strip()
    ])


def normalize_ltv_error_steps(raw_steps) -> list[str]:
    if raw_steps is None:
        return []

    if not isinstance(raw_steps, list):
        raw_steps = [raw_steps]

    steps = []
    for raw_step in raw_steps:
        if not str(raw_step).strip():
            continue

        step = _clean_step_text(str(raw_step))
        if _NUMBERED_STEP_PREFIX_RE.match(step):
            steps.extend(split_numbered_steps(step))
        else:
            steps.append(_remove_step_number(step))

    return _filter_empty_steps(steps)


def format_ltv_error_procedure(error_procedure: dict) -> dict:
    code = str(error_procedure.get("code", "")).strip()
    description = str(error_procedure.get("description", "")).strip()
    needs_resolved = bool(error_procedure.get("needs_resolved", False))
    name = f"{description} [{code}]" if description else f"[{code}]"
    tasks = [
        {"name": step, "description": "", "steps": []}
        for step in normalize_ltv_error_steps(error_procedure.get("procedures", []))
    ]

    return {
        "name": name,
        "description": f"LTV fault code {code}",
        "category": "LTV Error",
        "duration": "TBD",
        "ltv_error_code": code,
        "needs_resolved": needs_resolved,
        "tasks": tasks,
    }


def format_ltv_error_procedures(payload: dict) -> list[dict]:
    error_procedures = payload.get("error_procedures", [])
    if not isinstance(error_procedures, list):
        return []

    return [
        format_ltv_error_procedure(error_procedure)
        for error_procedure in error_procedures
        if isinstance(error_procedure, dict)
    ]


async def poll_ltv_error_procedures(
    upsert_procedure: ProcedureUpsert,
    url: str = LTV_ERRORS_URL,
    interval_seconds: int = LTV_ERROR_POLL_INTERVAL_SECONDS,
):
    while True:
        try:
            response = await asyncio.to_thread(requests.get, url, timeout=2)
            if response.status_code == 200:
                for procedure in format_ltv_error_procedures(response.json()):
                    await upsert_procedure(procedure)
            else:
                logging.warning(
                    "LTV errors request failed with status %s", response.status_code
                )
        except requests.RequestException:
            logging.exception("LTV errors request failed")
        except ValueError:
            logging.exception("LTV errors response was not valid JSON")

        await asyncio.sleep(interval_seconds)
