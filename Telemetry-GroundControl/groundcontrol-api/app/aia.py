from app.event import Event
from pydantic import BaseModel


AIA_MESSAGE_EVENT_TYPE = "aia_message"


class AiaMessage(BaseModel):
    message: str
    source: str = "aia"
    target: str = "hololens"


def create_aia_message_event(message: AiaMessage):
    return Event.create_event(
        AIA_MESSAGE_EVENT_TYPE,
        message.dict(exclude_none=True),
    )
