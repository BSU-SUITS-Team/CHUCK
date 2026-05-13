from __future__ import annotations

from threading import get_ident

from rich.markdown import Markdown
from rich.text import Text
from textual import events, on
from textual.app import App, ComposeResult
from textual.containers import VerticalScroll
from textual.widgets import Footer, Header, Input, LoadingIndicator, Static

from llm_cli_chat.agent import FastAgentChatBackend
from llm_cli_chat.voice import VoiceInputConfig, VoiceInputController


class ChatApp(App[None]):
    """Minimal Textual chat UI."""

    CSS = """
    Screen {
        layout: vertical;
    }

    #chat-log {
        height: 1fr;
        border: round $primary;
        padding: 0 1;
    }

    .speaker {
        height: auto;
        margin-top: 1;
    }

    .message {
        height: auto;
        margin-bottom: 1;
    }

    .system-message {
        height: auto;
        color: $text-muted;
    }

    #status {
        height: 1;
        color: $text-muted;
        padding: 0 1;
    }

    #prompt {
        height: 3;
        margin: 0 1 1 1;
    }

    #voice-help {
        height: 1;
        margin: 0 1;
    }

    #voice-spinner {
        height: 1;
        margin: 0 1;
    }

    #voice-transcript {
        height: 1;
        margin: 0 1 1 1;
        color: $text-muted;
    }
    """

    BINDINGS = [
        ("ctrl+c", "quit", "Quit"),
        ("ctrl+l", "clear_log", "Clear"),
    ]

    def __init__(
        self,
        backend: FastAgentChatBackend,
        *,
        voice_enabled: bool = False,
        voice_config: VoiceInputConfig | None = None,
        voice_model: object | None = None,
    ) -> None:
        super().__init__()
        self.backend = backend
        self.voice_enabled = voice_enabled
        self.voice_config = voice_config or VoiceInputConfig()
        self.voice_model = voice_model
        self.voice_controller: VoiceInputController | None = None
        self._app_thread_id: int | None = None

    def compose(self) -> ComposeResult:
        yield Header(show_clock=True)
        yield VerticalScroll(id="chat-log")
        yield Static("Starting agent...", id="status")
        if self.voice_enabled:
            yield Static("Press Space to start recording.", id="voice-help")
            yield LoadingIndicator(id="voice-spinner")
            yield Static("", id="voice-transcript")
        else:
            yield Input(placeholder="Type a message and press Enter...", id="prompt")
        yield Footer()

    async def on_mount(self) -> None:
        self._app_thread_id = get_ident()
        if self.voice_enabled:
            self.query_one("#voice-spinner", LoadingIndicator).visible = False
            self.query_one("#voice-transcript", Static).display = False

        try:
            await self.backend.start()
        except Exception as exc:  # noqa: BLE001 - keep startup failures visible in the TUI.
            self._set_text_input_enabled(False)
            self._status("Startup failed")
            self._log_system(f"Agent startup error: {exc}")
            return

        self._status("Ready")
        if self.voice_enabled:
            self._log_system("Agent ready. Press Space to start recording.")
            self._start_voice_input()
        else:
            self._log_system("Agent ready. Type a message to start chatting.")
            self.query_one("#prompt", Input).focus()

    async def on_unmount(self) -> None:
        if self.voice_controller is not None:
            self.voice_controller.stop()
        await self.backend.stop()

    @on(Input.Submitted, "#prompt")
    async def on_prompt_submitted(self, event: Input.Submitted) -> None:
        prompt = event.value.strip()
        if not prompt:
            return

        prompt_input = event.input
        prompt_input.value = ""
        await self._send_prompt(prompt)

    def on_key(self, event: events.Key) -> None:
        if not self.voice_enabled or event.key != "space":
            return

        event.prevent_default()
        event.stop()
        self._toggle_voice_recording()

    async def action_clear_log(self) -> None:
        await self.query_one("#chat-log", VerticalScroll).remove_children()
        self._log_system("Chat cleared.")

    async def _send_prompt(self, prompt: str) -> None:
        self._set_text_input_enabled(False)
        self._log_user(prompt)
        self._status("Streaming assistant response...")
        assistant_message = self._start_assistant_message()
        response = ""

        try:
            async for chunk in self.backend.stream(prompt):
                response += chunk
                self._update_assistant_message(assistant_message, response)
        except Exception as exc:  # noqa: BLE001 - surface backend errors in the TUI.
            self._log_system(f"Agent error: {exc}")
            self._status("Error")
        else:
            if not response:
                self._update_assistant_message(assistant_message, "(no response)")
            self._status("Ready")
        finally:
            self._set_text_input_enabled(True)

    def _start_voice_input(self) -> None:
        self.voice_controller = VoiceInputController(
            self.voice_config,
            model=self.voice_model,
            on_ready=lambda message: self._voice_dispatch(self._voice_ready, message),
            on_recording_started=lambda: self._voice_dispatch(self._voice_recording_started),
            on_realtime_transcript=lambda text: self._voice_dispatch(
                self._voice_realtime_transcript,
                text,
            ),
            on_recording_stopped=lambda: self._voice_dispatch(self._voice_recording_stopped),
            on_final_transcript=lambda text: self._voice_dispatch(
                self._voice_final_transcript,
                text,
            ),
            on_error=lambda message: self._voice_dispatch(self._voice_error, message),
        )
        self.voice_controller.start()

    def _toggle_voice_recording(self) -> None:
        if self.voice_controller is None:
            return

        self.voice_controller.toggle_recording()

    def _voice_dispatch(self, callback, *args: object) -> None:
        if self._app_thread_id == get_ident():
            callback(*args)
            return

        try:
            self.call_from_thread(callback, *args)
        except RuntimeError:
            pass

    def _voice_ready(self, message: str) -> None:
        self._set_voice_spinner(False)
        self._set_voice_help(message)
        self._status("Ready")

    def _voice_recording_started(self) -> None:
        self._set_voice_spinner(True)
        self._set_voice_help("Recording. Press Space again to stop and send.")
        self._set_voice_transcript("")
        self._status("Listening and transcribing...")

    def _voice_realtime_transcript(self, text: str) -> None:
        self._set_voice_transcript(text)

    def _voice_recording_stopped(self) -> None:
        self._set_voice_spinner(True)
        self._set_voice_help("Transcribing voice input...")
        self._status("Transcribing voice input...")

    def _voice_final_transcript(self, text: str) -> None:
        self._set_voice_spinner(False)
        self._set_voice_help("Press Space to start recording.")
        self._set_voice_transcript(text)
        self.run_worker(self._send_prompt(text), exclusive=False)

    def _voice_error(self, message: str) -> None:
        self._set_voice_spinner(False)
        self._set_voice_help("Press Space to start recording.")
        self._status("Voice error")
        self._log_system(message)

    def _log_user(self, message: str) -> None:
        self._append_chat_widget(Static(Text("You", style="bold cyan"), classes="speaker"))
        self._append_chat_widget(Static(message, classes="message"))

    def _log_assistant(self, message: str) -> None:
        assistant_message = self._start_assistant_message()
        self._update_assistant_message(assistant_message, message)

    def _log_system(self, message: str) -> None:
        self._append_chat_widget(Static(message, classes="system-message"))

    def _start_assistant_message(self) -> Static:
        self._append_chat_widget(Static(Text("Assistant", style="bold green"), classes="speaker"))
        return self._append_chat_widget(Static("", classes="message"))

    def _update_assistant_message(self, widget: Static, message: str) -> None:
        widget.update(Markdown(message))
        self._scroll_chat_to_end()

    def _append_chat_widget(self, widget: Static) -> Static:
        self.query_one("#chat-log", VerticalScroll).mount(widget)
        self._scroll_chat_to_end()
        return widget

    def _scroll_chat_to_end(self) -> None:
        self.query_one("#chat-log", VerticalScroll).scroll_end(animate=False)

    def _status(self, message: str) -> None:
        self.query_one("#status", Static).update(message)

    def _set_text_input_enabled(self, enabled: bool) -> None:
        if self.voice_enabled:
            return

        prompt = self.query_one("#prompt", Input)
        prompt.disabled = not enabled
        if enabled:
            prompt.focus()

    def _set_voice_spinner(self, visible: bool) -> None:
        if self.voice_enabled:
            self.query_one("#voice-spinner", LoadingIndicator).visible = visible

    def _set_voice_help(self, message: str) -> None:
        if self.voice_enabled:
            self.query_one("#voice-help", Static).update(message)

    def _set_voice_transcript(self, message: str) -> None:
        if self.voice_enabled:
            transcript = self.query_one("#voice-transcript", Static)
            transcript.display = bool(message)
            transcript.update(message)
