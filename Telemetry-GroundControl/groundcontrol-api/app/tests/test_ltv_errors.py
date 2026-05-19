from app.ltv_errors import (
    OLLAMA_STEP_TITLE_MODEL,
    _STEP_TITLE_CACHE,
    _ollama_generate_urls,
    format_ltv_error_procedure,
    format_ltv_error_procedures,
    generate_step_title_with_ollama,
    normalize_ltv_error_steps,
    split_numbered_steps,
)


def title_generator(
    step_body: str,
    procedure_name: str,
    step_index: int,
    total_steps: int,
) -> str:
    return f"Generated Title {step_index}"


def test_split_numbered_steps_from_merged_step():
    merged_step = (
        "1. Locate the NAV Control Panel 2. Flip the primary control switch "
        "3. Identify the LIDAR RESET button 4. Set the dial to 1.5 "
        "17.To test the software reboot"
    )

    assert split_numbered_steps(merged_step) == [
        "Locate the NAV Control Panel",
        "Flip the primary control switch",
        "Identify the LIDAR RESET button",
        "Set the dial to 1.5",
        "To test the software reboot",
    ]


def test_normalize_ltv_error_steps_keeps_explicit_step_list():
    assert normalize_ltv_error_steps(["1. Locate the dust sensor", "2. Replace it"]) == [
        "Locate the dust sensor",
        "Replace it",
    ]


def test_normalize_ltv_error_steps_filters_empty_steps():
    assert normalize_ltv_error_steps(["1. ", "", "2. Locate the dust sensor"]) == [
        "Locate the dust sensor",
    ]
    assert split_numbered_steps("1. 2. Locate the dust sensor 3. ") == [
        "Locate the dust sensor",
    ]


def test_format_ltv_error_procedure_uses_ground_control_shape():
    procedure = format_ltv_error_procedure(
        {
            "code": "2235",
            "description": "Dust Sensor Error",
            "needs_resolved": True,
            "procedures": ["1. Locate the dust sensor", "2. Replace the sensor"],
        },
        title_generator=title_generator,
    )

    assert procedure["name"] == "Dust Sensor Error [2235]"
    assert procedure["category"] == "LTV Error"
    assert procedure["ltv_error_code"] == "2235"
    assert procedure["needs_resolved"] is True
    assert procedure["tasks"] == [
        {
            "name": "Generated Title 1",
            "description": "",
            "steps": [
                {
                    "type": "text",
                    "body": "Locate the dust sensor",
                }
            ],
        },
        {
            "name": "Generated Title 2",
            "description": "",
            "steps": [
                {
                    "type": "text",
                    "body": "Replace the sensor",
                }
            ],
        }
    ]


def test_format_ltv_error_procedures_ignores_invalid_collection():
    assert format_ltv_error_procedures({"error_procedures": {}}) == []


def test_format_ltv_error_procedure_logs_generation_progress(capsys):
    format_ltv_error_procedure(
        {
            "code": "2235",
            "description": "Dust Sensor Error",
            "procedures": ["1. Locate the dust sensor"],
        },
        title_generator=title_generator,
    )

    captured = capsys.readouterr()
    assert "Generating 1 new step title(s) for Dust Sensor Error [2235]" in captured.out
    assert "Requesting title 1/1 for Dust Sensor Error [2235]" in captured.out
    assert "Generated title 1/1 for Dust Sensor Error [2235]" in captured.out


def test_generate_step_title_with_ollama_posts_short_title_prompt(monkeypatch):
    requests = []
    _STEP_TITLE_CACHE.clear()

    class Response:
        def raise_for_status(self):
            pass

        def json(self):
            return {"response": "Title: Inspect Dust Sensor."}

    def post(url, json, timeout):
        requests.append({"url": url, "json": json, "timeout": timeout})
        return Response()

    monkeypatch.setattr("app.ltv_errors.requests.post", post)

    assert (
        generate_step_title_with_ollama(
            "Locate the dust sensor",
            "Dust Sensor Error [2235]",
            1,
            2,
        )
        == "Inspect Dust Sensor"
    )
    assert requests[0]["url"].endswith("/api/generate")
    assert requests[0]["json"]["stream"] is False
    assert requests[0]["json"]["think"] is False
    assert requests[0]["json"]["options"]["temperature"] == 0.1
    assert "Return only the title" in requests[0]["json"]["prompt"]


def test_default_ollama_urls_prefer_docker_host_inside_container(monkeypatch):
    monkeypatch.delenv("OLLAMA_GENERATE_URL", raising=False)
    monkeypatch.delenv("OLLAMA_ENDPOINT", raising=False)
    monkeypatch.delenv("OLLAMA_HOST", raising=False)
    monkeypatch.setattr("app.ltv_errors.exists", lambda path: path == "/.dockerenv")

    assert _ollama_generate_urls() == [
        "http://host.docker.internal:11434/api/generate",
        "http://172.17.0.1:11434/api/generate",
        "http://localhost:11434/api/generate",
    ]


def test_format_ltv_error_procedure_does_not_log_cached_titles(monkeypatch, capsys):
    _STEP_TITLE_CACHE.clear()
    monkeypatch.setenv("OLLAMA_GENERATE_URL", "http://ollama.test/api/generate")
    _STEP_TITLE_CACHE[
        (
            "http://ollama.test/api/generate",
            OLLAMA_STEP_TITLE_MODEL,
            "Locate the dust sensor",
        )
    ] = "Cached Title"

    procedure = format_ltv_error_procedure(
        {
            "code": "2235",
            "description": "Dust Sensor Error",
            "procedures": ["1. Locate the dust sensor"],
        }
    )

    captured = capsys.readouterr()
    assert captured.out == ""
    assert procedure["tasks"][0]["name"] == "Cached Title"
