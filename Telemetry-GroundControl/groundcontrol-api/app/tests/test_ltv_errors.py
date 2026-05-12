from app.ltv_errors import (
    format_ltv_error_procedure,
    format_ltv_error_procedures,
    normalize_ltv_error_steps,
    split_numbered_steps,
)


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


def test_format_ltv_error_procedure_uses_ground_control_shape():
    procedure = format_ltv_error_procedure(
        {
            "code": "2235",
            "description": "Dust Sensor Error",
            "needs_resolved": True,
            "procedures": ["1. Locate the dust sensor", "2. Replace the sensor"],
        }
    )

    assert procedure["name"] == "Dust Sensor Error [2235]"
    assert procedure["category"] == "LTV Error"
    assert procedure["ltv_error_code"] == "2235"
    assert procedure["needs_resolved"] is True
    assert procedure["tasks"] == [
        {
            "name": "Locate the dust sensor",
            "description": "",
            "steps": [],
        },
        {
            "name": "Replace the sensor",
            "description": "",
            "steps": [],
        }
    ]


def test_format_ltv_error_procedures_ignores_invalid_collection():
    assert format_ltv_error_procedures({"error_procedures": {}}) == []
