import pytest

from app.datastore import Datastore
from app.routers import voice as voice_router
from app.voice import (
    VOICE_TRANSCRIPTION_EVENT_TYPE,
    VoiceTranscriptionCommand,
    VoiceTranscriptionToggleState,
    create_voice_transcription_event,
)


class FakeDatastore:
    def __init__(self):
        self.events = []

    async def add_event(self, key, event):
        self.events.append((key, event))


def test_create_voice_transcription_event_uses_command_event_type():
    event = create_voice_transcription_event(
        VoiceTranscriptionCommand(),
        transcribing=True,
    )

    assert event["type"] == VOICE_TRANSCRIPTION_EVENT_TYPE
    assert event["data"]["action"] == "toggle"
    assert event["data"]["target"] == "aia"
    assert event["data"]["source"] == "hololens"
    assert event["data"]["transcribing"] is True


def test_voice_transcription_toggle_state_flips_each_call():
    state = VoiceTranscriptionToggleState()

    assert state.toggle() is True
    assert state.toggle() is False


@pytest.mark.asyncio
async def test_toggle_voice_transcription_endpoint_enqueues_event(monkeypatch):
    fake_datastore = FakeDatastore()
    state = VoiceTranscriptionToggleState()
    monkeypatch.setattr(voice_router, "ds", fake_datastore)
    monkeypatch.setattr(voice_router, "voice_transcription_state", state)

    response = await voice_router.toggle_voice_transcription()

    assert response["message"] == "Voice transcription toggled"
    assert response["transcribing"] is True
    assert fake_datastore.events == [
        (VOICE_TRANSCRIPTION_EVENT_TYPE, response["event"]),
    ]
    assert response["event"]["data"]["action"] == "toggle"
    assert response["event"]["data"]["transcribing"] is True


@pytest.mark.asyncio
async def test_voice_transcription_events_are_cached_for_startup_replay():
    datastore = Datastore()
    event = create_voice_transcription_event(
        VoiceTranscriptionCommand(),
        transcribing=True,
    )

    await datastore.add_event(VOICE_TRANSCRIPTION_EVENT_TYPE, event)
    event_stream = datastore.make_async_gen()

    assert await event_stream.__anext__() == event
    assert await datastore.get_all() == [event]

    await event_stream.aclose()
