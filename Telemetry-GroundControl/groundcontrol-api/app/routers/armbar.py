from app.armbar import (
    ARM_BAR_BUTTON_PRESS_EVENT_TYPE,
    ArmbarButtonPress,
    create_armbar_button_press_event,
)
from app.datastore import ds
from fastapi import APIRouter


router = APIRouter(prefix="/armbar", tags=["armbar"])


@router.post("/button-presses")
async def send_armbar_button_press(press: ArmbarButtonPress):
    event = create_armbar_button_press_event(press)
    await ds.add_event(ARM_BAR_BUTTON_PRESS_EVENT_TYPE, event)
    return {"message": "Armbar button press sent", "event": event}


@router.post("/key-presses")
async def send_armbar_key_press(press: ArmbarButtonPress):
    return await send_armbar_button_press(press)
