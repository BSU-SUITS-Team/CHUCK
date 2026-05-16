from typing import Optional

from app.datastore import ds
from app.voice import (
    VOICE_TRANSCRIPTION_EVENT_TYPE,
    VoiceTranscriptionCommand,
    create_voice_transcription_event,
    voice_transcription_state,
)
from fastapi import APIRouter


router = APIRouter(prefix="/voice", tags=["voice"])


@router.post("/transcription/toggle")
async def toggle_voice_transcription(command: Optional[VoiceTranscriptionCommand] = None):
    command = command or VoiceTranscriptionCommand()
    transcribing = voice_transcription_state.toggle()
    event = create_voice_transcription_event(command, transcribing)
    await ds.add_event(VOICE_TRANSCRIPTION_EVENT_TYPE, event)
    return {
        "message": "Voice transcription toggled",
        "transcribing": transcribing,
        "event": event,
    }
