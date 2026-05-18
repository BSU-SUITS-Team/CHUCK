import pytest
from pydantic import ValidationError

from app.armbar import (
    ARM_BAR_BUTTON_PRESS_EVENT_TYPE,
    ArmbarButtonPress,
    create_armbar_button_press_event,
)
from app.datastore import Datastore
from app.routers import armbar as armbar_router


class FakeDatastore:
    def __init__(self):
        self.events = []

    async def add_event(self, key, event):
        self.events.append((key, event))


@pytest.mark.parametrize("key", ["1", "B2", "button3"])
def test_armbar_button_press_accepts_key_aliases(key):
    press = ArmbarButtonPress(key=key)

    assert press.button == int(key[-1])
    assert press.key == str(press.button)


@pytest.mark.parametrize("key", ["0", "7", "left"])
def test_armbar_button_press_rejects_invalid_keys(key):
    with pytest.raises(ValidationError):
        ArmbarButtonPress(key=key)


def test_create_armbar_button_press_event_uses_event_type():
    event = create_armbar_button_press_event(ArmbarButtonPress(button=4))

    assert event["type"] == ARM_BAR_BUTTON_PRESS_EVENT_TYPE
    assert event["data"]["action"] == "press"
    assert event["data"]["button"] == 4
    assert event["data"]["key"] == "4"
    assert event["data"]["target"] == "hololens"
    assert event["data"]["source"] == "armbar"


@pytest.mark.asyncio
async def test_send_armbar_button_press_endpoint_enqueues_event(monkeypatch):
    fake_datastore = FakeDatastore()
    monkeypatch.setattr(armbar_router, "ds", fake_datastore)

    response = await armbar_router.send_armbar_button_press(ArmbarButtonPress(button=2))

    assert response["message"] == "Armbar button press sent"
    assert fake_datastore.events == [
        (ARM_BAR_BUTTON_PRESS_EVENT_TYPE, response["event"]),
    ]
    assert response["event"]["data"]["button"] == 2


@pytest.mark.asyncio
async def test_armbar_button_press_events_are_cached_for_startup_replay():
    datastore = Datastore()
    event = create_armbar_button_press_event(ArmbarButtonPress(button=6))

    await datastore.add_event(ARM_BAR_BUTTON_PRESS_EVENT_TYPE, event)
    event_stream = datastore.make_async_gen()

    assert await event_stream.__anext__() == event
    assert await datastore.get_all() == [event]

    await event_stream.aclose()
