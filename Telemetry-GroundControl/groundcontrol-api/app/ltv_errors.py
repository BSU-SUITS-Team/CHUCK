import asyncio
import logging
import re
from os import getenv
from os.path import exists
from typing import Awaitable, Callable

import requests


LTV_ERRORS_URL = getenv(
    "LTV_ERRORS_ENDPOINT",
    f"{getenv('TSS_ENDPOINT', 'http://localhost:14141')}/data/LTV_ERRORS.json",
)
LTV_ERROR_POLL_INTERVAL_SECONDS = 3
OLLAMA_STEP_TITLE_MODEL = getenv(
    "OLLAMA_STEP_TITLE_MODEL",
    getenv("OLLAMA_MODEL", "qwen3.6:35b-a3b"),
)
OLLAMA_STEP_TITLE_TIMEOUT_SECONDS = float(
    getenv("OLLAMA_STEP_TITLE_TIMEOUT_SECONDS", "10")
)

_NUMBERED_STEP_BOUNDARY_RE = re.compile(r"\s+(?=\d+\.(?=\s|[^\d\s]|$))")
_NUMBERED_STEP_PREFIX_RE = re.compile(r"^\d+\.(?=\s|[^\d\s]|$)")
_MISSING_STEP_SPACE_RE = re.compile(r"^(\d+)\.(?=[^\d\s])")
_LEADING_STEP_NUMBER_RE = re.compile(r"^\d+\.(?:\s+|(?=[^\d\s])|$)")
_TITLE_PREFIX_RE = re.compile(r"^(?:step\s+title|title)\s*:\s*", re.IGNORECASE)
_THINKING_RE = re.compile(r"<think>.*?</think>", re.IGNORECASE | re.DOTALL)


ProcedureUpsert = Callable[[dict], Awaitable[bool]]
StepTitleGenerator = Callable[[str, str, int, int], str]
_STEP_TITLE_CACHE: dict[tuple[str, str, str], str] = {}


def _clean_step_text(step_text: str) -> str:
    normalized = re.sub(r"\s+", " ", step_text).strip()
    return _MISSING_STEP_SPACE_RE.sub(r"\1. ", normalized)


def _remove_step_number(step_text: str) -> str:
    return _LEADING_STEP_NUMBER_RE.sub("", step_text).strip()


def _filter_empty_steps(steps: list[str]) -> list[str]:
    return [step for step in steps if step.strip()]


def _ollama_generate_urls() -> list[str]:
    explicit_url = getenv("OLLAMA_GENERATE_URL")
    if explicit_url:
        return [explicit_url]

    base_url = (
        getenv("OLLAMA_ENDPOINT")
        or getenv("OLLAMA_HOST")
    )
    if base_url:
        if not base_url.startswith(("http://", "https://")):
            base_url = f"http://{base_url}"

        return [f"{base_url.rstrip('/')}/api/generate"]

    base_urls = ["http://localhost:11434"]
    if exists("/.dockerenv"):
        base_urls = [
            "http://host.docker.internal:11434",
            "http://172.17.0.1:11434",
            *base_urls,
        ]

    return [f"{base_url}/api/generate" for base_url in base_urls]


def _log_title_generation(message: str) -> None:
    print(f"[ltv-errors] {message}", flush=True)


def _cached_step_title(step_body: str) -> str | None:
    for url in _ollama_generate_urls():
        cache_key = (url, OLLAMA_STEP_TITLE_MODEL, step_body)
        if cache_key in _STEP_TITLE_CACHE:
            return _STEP_TITLE_CACHE[cache_key]

    return None


def _title_prompt(
    step_body: str,
    procedure_name: str,
    step_index: int,
    total_steps: int,
) -> str:
    return (
        "Create a short, professional procedure-step title for the step below.\n"
        "Rules:\n"
        "- Return only the title.\n"
        "- Use 2 to 6 words.\n"
        "- Use title case.\n"
        "- Do not include numbering, quotes, punctuation, or commentary.\n\n"
        f"Procedure: {procedure_name}\n"
        f"Step {step_index} of {total_steps}: {step_body}\n"
        "Title:"
    )


def _sanitize_generated_title(title: str) -> str:
    title = _THINKING_RE.sub("", title)
    title = re.sub(r"\s+", " ", title).strip()
    title = title.strip("\"'`*-. ")
    title = _TITLE_PREFIX_RE.sub("", title).strip()
    title = title.rstrip(".:;,- ")

    words = title.split()
    if len(words) > 8:
        title = " ".join(words[:8]).rstrip(".:;,- ")

    return title[:80].strip()


def _fallback_step_title(step_body: str, step_index: int) -> str:
    step_title = re.split(r"[,;:.(\[]", step_body, maxsplit=1)[0].strip()
    step_title = re.sub(r"\s+", " ", step_title).strip("\"'`*-. ")
    words = step_title.split()
    if not words:
        return f"Step {step_index}"

    return " ".join(words[:6]).rstrip(".:;,- ")


def generate_step_title_with_ollama(
    step_body: str,
    procedure_name: str,
    step_index: int,
    total_steps: int,
) -> str:
    cached_title = _cached_step_title(step_body)
    if cached_title is not None:
        return cached_title

    payload = {
        "model": OLLAMA_STEP_TITLE_MODEL,
        "prompt": _title_prompt(
            step_body,
            procedure_name,
            step_index,
            total_steps,
        ),
        "stream": False,
        "think": False,
        "options": {
            "num_predict": 20,
            "temperature": 0.1,
        },
    }

    last_error: Exception | None = None
    for url in _ollama_generate_urls():
        cache_key = (url, OLLAMA_STEP_TITLE_MODEL, step_body)
        _log_title_generation(
            f"Sending Ollama request {step_index}/{total_steps} for "
            f"{procedure_name} to {url}"
        )
        try:
            response = requests.post(
                url,
                json=payload,
                timeout=OLLAMA_STEP_TITLE_TIMEOUT_SECONDS,
            )
            response.raise_for_status()

            generated_title = _sanitize_generated_title(
                str(response.json().get("response", ""))
            )
            if not generated_title:
                raise ValueError("Ollama returned an empty step title")

            _STEP_TITLE_CACHE[cache_key] = generated_title
            return generated_title
        except requests.RequestException as exc:
            last_error = exc
            _log_title_generation(
                f"Ollama request failed for {url}: {exc}; trying next endpoint"
            )

    if last_error is not None:
        raise last_error
    raise ValueError("No Ollama endpoints configured")


def _generate_step_title(
    step_body: str,
    procedure_name: str,
    step_index: int,
    total_steps: int,
    title_generator: StepTitleGenerator,
) -> str:
    try:
        title = title_generator(step_body, procedure_name, step_index, total_steps)
        title = _sanitize_generated_title(str(title))
        if title:
            return title
    except Exception as exc:
        logging.warning(
            "LTV error step title generation failed for %s step %s",
            procedure_name,
            step_index,
            exc_info=True,
        )
        _log_title_generation(
            f"Title generation failed for {procedure_name} step "
            f"{step_index}/{total_steps}: {exc}; using fallback"
        )

    return _fallback_step_title(step_body, step_index)


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


def format_ltv_error_procedure(
    error_procedure: dict,
    title_generator: StepTitleGenerator | None = None,
) -> dict:
    code = str(error_procedure.get("code", "")).strip()
    description = str(error_procedure.get("description", "")).strip()
    needs_resolved = bool(error_procedure.get("needs_resolved", False))
    name = f"{description} [{code}]" if description else f"[{code}]"
    parsed_steps = normalize_ltv_error_steps(error_procedure.get("procedures", []))
    is_default_title_generator = title_generator is None
    steps_needing_titles = [
        step_body
        for step_body in parsed_steps
        if not is_default_title_generator or _cached_step_title(step_body) is None
    ]
    title_source = (
        f"Ollama model {OLLAMA_STEP_TITLE_MODEL}"
        if is_default_title_generator
        else "custom title generator"
    )
    title_generator = title_generator or generate_step_title_with_ollama
    tasks = []

    if steps_needing_titles:
        _log_title_generation(
            f"Generating {len(steps_needing_titles)} new step title(s) for {name} "
            f"with {title_source}"
        )

    for index, step_body in enumerate(parsed_steps, start=1):
        should_log_step = (
            not is_default_title_generator or _cached_step_title(step_body) is None
        )
        if should_log_step:
            _log_title_generation(
                f"Requesting title {index}/{len(parsed_steps)} for {name}"
            )
        step_title = _generate_step_title(
            step_body,
            name,
            index,
            len(parsed_steps),
            title_generator,
        )
        if should_log_step:
            _log_title_generation(
                f"Generated title {index}/{len(parsed_steps)} for {name}: {step_title}"
            )
        tasks.append({
            "name": step_title,
            "description": "",
            "steps": [
                {
                    "type": "text",
                    "body": step_body,
                }
            ],
        })

    return {
        "name": name,
        "description": f"LTV fault code {code}",
        "category": "LTV Error",
        "duration": "TBD",
        "ltv_error_code": code,
        "needs_resolved": needs_resolved,
        "tasks": tasks,
    }


def format_ltv_error_procedures(
    payload: dict,
    title_generator: StepTitleGenerator | None = None,
) -> list[dict]:
    error_procedures = payload.get("error_procedures", [])
    if not isinstance(error_procedures, list):
        return []

    return [
        format_ltv_error_procedure(error_procedure, title_generator)
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
                procedures = await asyncio.to_thread(
                    format_ltv_error_procedures,
                    response.json(),
                )
                for procedure in procedures:
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
