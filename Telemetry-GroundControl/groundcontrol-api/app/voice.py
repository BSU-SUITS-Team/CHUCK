from typing import Literal

from app.event import Event
from pydantic import BaseModel


VoiceTranscriptionAction = Literal["toggle"]
VOICE_TRANSCRIPTION_EVENT_TYPE = "voice_transcription_command"


class VoiceTranscriptionCommand(BaseModel):
    action: VoiceTranscriptionAction = "toggle"
    target: str = "aia"
    source: str = "hololens"


class VoiceTranscriptionToggleState:
    def __init__(self):
        self.transcribing = False

    def toggle(self):
        self.transcribing = not self.transcribing
        return self.transcribing


voice_transcription_state = VoiceTranscriptionToggleState()


def create_voice_transcription_event(command: VoiceTranscriptionCommand, transcribing: bool):
    return Event.create_event(
        VOICE_TRANSCRIPTION_EVENT_TYPE,
        {
            **command.dict(exclude_none=True),
            "transcribing": transcribing,
        },
    )
