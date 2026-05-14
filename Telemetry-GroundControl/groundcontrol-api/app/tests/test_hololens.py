import pytest
from pydantic import ValidationError

from app.datastore import Datastore
from app.hololens import (
    HOLOLENS_COMMAND_EVENT_TYPE,
    HololensCommand,
    create_hololens_command_event,
)


def test_hololens_window_commands_require_window():
    with pytest.raises(ValidationError):
        HololensCommand(action="open_window")

    with pytest.raises(ValidationError):
        HololensCommand(action="close_window")


def test_hololens_procedure_commands_require_procedure():
    with pytest.raises(ValidationError):
        HololensCommand(action="open_procedure")


def test_create_hololens_command_event_uses_command_event_type():
    event = create_hololens_command_event(
        HololensCommand(action="open_window", window="navigation")
    )

    assert event["type"] == HOLOLENS_COMMAND_EVENT_TYPE
    assert event["data"]["action"] == "open_window"
    assert event["data"]["window"] == "navigation"
    assert event["data"]["target"] == "hololens"


@pytest.mark.asyncio
async def test_hololens_command_events_are_cached_for_startup_replay():
    datastore = Datastore()
    event = create_hololens_command_event(
        HololensCommand(action="open_window", window="navigation")
    )

    await datastore.add_event(HOLOLENS_COMMAND_EVENT_TYPE, event)
    event_stream = datastore.make_async_gen()

    assert await event_stream.__anext__() == event
    assert await datastore.get_all() == [event]

    await event_stream.aclose()
