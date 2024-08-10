from fastapi import APIRouter
from app.datastore import ds
from app.event import Event
from pydantic import BaseModel
import asyncio

router = APIRouter(prefix="/notifications", tags=["notifications"])


async def add_notification_to_ds(notification):
    notification_event = Event.create_event(
        "notification", notification.to_dict())
    await ds.add_event("notification", notification_event)


class Message(BaseModel):
    content: str
    severity: int

    def to_dict(self):
        return {"content": self.content, "severity": self.severity}


@router.post("/")
def post_message(message: Message):
    """Sends a message to a user and returns 200 if successful.

    Severity Levels:
    - 0: Critical warning
    - 1: Warning
    - 2: Info
    """
    asyncio.run(add_notification_to_ds(message))
    return 200
