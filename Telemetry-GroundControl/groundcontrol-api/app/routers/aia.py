from app.aia import AIA_MESSAGE_EVENT_TYPE, AiaMessage, create_aia_message_event
from app.datastore import ds
from fastapi import APIRouter


router = APIRouter(prefix="/aia", tags=["aia"])


@router.post("/messages")
async def send_aia_message(message: AiaMessage):
    event = create_aia_message_event(message)
    await ds.add_event(AIA_MESSAGE_EVENT_TYPE, event)
    return {"message": "AIA message sent", "event": event}
