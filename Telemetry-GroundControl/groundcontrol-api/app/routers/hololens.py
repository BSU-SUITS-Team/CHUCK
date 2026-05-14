from app.datastore import ds
from app.hololens import (
    HOLOLENS_COMMAND_EVENT_TYPE,
    HololensCommand,
    create_hololens_command_event,
)
from fastapi import APIRouter


router = APIRouter(prefix="/hololens", tags=["hololens"])


@router.post("/commands")
async def send_hololens_command(command: HololensCommand):
    event = create_hololens_command_event(command)
    await ds.add_event(HOLOLENS_COMMAND_EVENT_TYPE, event)
    return {"message": "Hololens command sent", "event": event}
