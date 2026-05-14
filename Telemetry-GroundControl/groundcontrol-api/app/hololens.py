from typing import Literal, Optional

from app.event import Event
from pydantic import BaseModel, root_validator


HololensCommandAction = Literal["open_window", "close_window", "open_procedure"]
HOLOLENS_COMMAND_EVENT_TYPE = "hololens_command"


class HololensCommand(BaseModel):
    action: HololensCommandAction
    window: Optional[str] = None
    procedure: Optional[str] = None
    target: str = "hololens"
    source: str = "LMCC"

    @root_validator
    def require_command_target(cls, values):
        action = values.get("action")
        window = values.get("window")
        procedure = values.get("procedure")

        if action in {"open_window", "close_window"} and not window:
            raise ValueError("window is required for window commands")
        if action == "open_procedure" and not procedure:
            raise ValueError("procedure is required for procedure commands")

        return values


def create_hololens_command_event(command: HololensCommand):
    return Event.create_event(
        HOLOLENS_COMMAND_EVENT_TYPE,
        command.dict(exclude_none=True),
    )
