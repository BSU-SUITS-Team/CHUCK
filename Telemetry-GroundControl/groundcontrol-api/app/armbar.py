from typing import Literal, Optional

from app.event import Event
from pydantic import BaseModel, root_validator


ARM_BAR_BUTTON_PRESS_EVENT_TYPE = "armbar_button_press"
ArmbarButtonPressAction = Literal["press"]


class ArmbarButtonPress(BaseModel):
    action: ArmbarButtonPressAction = "press"
    button: int
    key: Optional[str] = None
    target: str = "hololens"
    source: str = "armbar"

    @root_validator(pre=True)
    def normalize_button(cls, values):
        raw_button = values.get("button")
        if raw_button is None:
            raw_button = values.get("key")

        if raw_button is None:
            return values

        button = cls._parse_button(raw_button)
        values["button"] = button
        values["key"] = str(button)
        return values

    @staticmethod
    def _parse_button(value):
        if isinstance(value, bool):
            raise ValueError("button must be an armbar button number from 1 to 6")

        text = str(value).strip().lower()
        if text.startswith("button"):
            text = text[len("button") :]
        elif text.startswith("b"):
            text = text[1:]

        try:
            button = int(text)
        except ValueError as exc:
            raise ValueError("button must be an armbar button number from 1 to 6") from exc

        if button < 1 or button > 6:
            raise ValueError("button must be an armbar button number from 1 to 6")

        return button


def create_armbar_button_press_event(press: ArmbarButtonPress):
    return Event.create_event(
        ARM_BAR_BUTTON_PRESS_EVENT_TYPE,
        press.dict(exclude_none=True),
    )
