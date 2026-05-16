import pytest

from app.aia import AIA_MESSAGE_EVENT_TYPE, AiaMessage, create_aia_message_event
from app.datastore import Datastore
from app.routers import aia as aia_router


class FakeDatastore:
    def __init__(self):
        self.events = []

    async def add_event(self, key, event):
        self.events.append((key, event))


def test_create_aia_message_event_uses_aia_message_type():
    event = create_aia_message_event(AiaMessage(message="Check oxygen pressure."))

    assert event["type"] == AIA_MESSAGE_EVENT_TYPE
    assert event["data"]["message"] == "Check oxygen pressure."
    assert event["data"]["source"] == "aia"
    assert event["data"]["target"] == "hololens"


@pytest.mark.asyncio
async def test_send_aia_message_endpoint_enqueues_event(monkeypatch):
    fake_datastore = FakeDatastore()
    monkeypatch.setattr(aia_router, "ds", fake_datastore)

    response = await aia_router.send_aia_message(AiaMessage(message="Proceed to step two."))

    assert response["message"] == "AIA message sent"
    assert fake_datastore.events == [
        (AIA_MESSAGE_EVENT_TYPE, response["event"]),
    ]
    assert response["event"]["data"]["message"] == "Proceed to step two."


@pytest.mark.asyncio
async def test_aia_message_events_are_cached_for_startup_replay():
    datastore = Datastore()
    event = create_aia_message_event(AiaMessage(message="Cached message."))

    await datastore.add_event(AIA_MESSAGE_EVENT_TYPE, event)
    event_stream = datastore.make_async_gen()

    assert await event_stream.__anext__() == event
    assert await datastore.get_all() == [event]

    await event_stream.aclose()
