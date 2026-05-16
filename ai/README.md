# LLM CLI Chat

A small Python chat app scaffold that uses:

- `textual` for the terminal UI
- `fast-agent` via the `fast-agent-mcp` package for the LLM agent layer
- `openai-whisper` for optional speech recognition

Text input is the default. Voice input is available behind a CLI flag.

## Setup

This project targets Python `>=3.13.5`, matching the current `fast-agent-mcp` requirement.

```bash
uv venv --python 3.13 .venv
source .venv/bin/activate
uv sync
```

The default model is Qwen3.6 through Ollama's OpenAI-compatible endpoint at
`http://localhost:11434/v1` via fast-agent's `generic` provider. Start Ollama
and make sure the model is available:

```bash
ollama serve
ollama pull qwen3.6:35b-a3b
```

The configured default model id is:

```bash
qwen3.6:35b-a3b
```

The app passes no-thinking request parameters for this default model. You can
confirm the active Ollama model id with:

```bash
curl http://localhost:11434/v1/models
```

You can override the default Ollama endpoint if needed:

```bash
cp .env.example .env
```

Then edit `.env` or export the variables in your shell.

## Run

```bash
uv run llm-chat
```

You can still override the model:

```bash
uv run llm-chat --model "generic.qwen3.6:35b-a3b?reasoning=off"
```

Inside the app, type a message and press Enter. Use `Ctrl+C` to quit and `Ctrl+L` to clear the chat log.
Assistant responses stream into the chat log as model chunks arrive.

Before each typed or voice prompt is sent to the model, the app fetches current mission context and prepends it to the model-visible message:

- available procedure names from `GROUND_CONTROL_API_URL` or `http://localhost:8181`

Current biometrics and full procedure text are fetched through model-callable
tools. The assistant can also issue Hololens display commands through
`GROUND_CONTROL_API_URL` to open or close windows and display a named procedure.
When the model starts one of those tool calls, the chat output shows a short
status message that matches the active tool, such as `Opening window...` or
`Checking current biometrics...`, before the final answer.

If either service is unavailable, chat still works and the injected context records that source as unavailable. You can override or disable this behavior:

```bash
uv run llm-chat --ground-control-api-url http://localhost:8181 --tss-endpoint http://localhost:14141
uv run llm-chat --user-eva eva1
uv run llm-chat --no-mission-context
```

## Voice Input

Voice mode replaces the text input box with a simple Space-toggle recorder:

```bash
uv run llm-chat --voice
```

Press Space to start recording. While recording, the app shows a spinner and reveals a compact live transcript only after speech is recognized. Space again stops recording and sends the collected transcript to the model.

Voice mode includes a microphone menu above the recorder controls. Choosing a
device uses that microphone for subsequent recordings. You can also set an
initial input device ID from the CLI:

```bash
uv run llm-chat --voice --audio-input-device 2
```

When voice mode is enabled, the app also listens to the Ground Control API
event websocket derived from `--ground-control-api-url`. A
`POST /voice/transcription/toggle` request on that API emits a voice command
event that toggles recording in the TUI.

By default, voice mode now uses the `whisper-stream` binary when it is available on `PATH` and `ggml-base.en.bin` exists in the working directory. This matches the low-repeat command:

```bash
whisper-stream -m ggml-base.en.bin -t 8 --step 500 --length 5000
```

The app launches that process on Space and reads transcription text from stdout while recording. If `whisper-stream` or the ggml model is unavailable, `--voice-engine auto` falls back to the Python Whisper engine.

You can force either engine:

```bash
uv run llm-chat --voice --voice-engine whisper-stream
uv run llm-chat --voice --voice-engine python
```

The project pins `openai-whisper==20250625` and defaults to Whisper's `tiny.en` speech model for the Python fallback. When the Python engine is used, voice mode loads the Whisper model before the TUI starts so model downloads and cache repairs happen outside Textual.

The Python fallback uses a committed-text plus live-draft model similar to `whisper.cpp`: each chunk keeps a small audio overlap, replaces the current draft, and periodically commits stable text. If no live transcript is ready yet, the app falls back to one final transcription pass.

Device selection defaults to `auto`, which uses CUDA when available and otherwise CPU. On this machine, Apple MPS is available, but the `tiny.en` benchmark was faster on CPU for short realtime chunks. You can still force a device:

```bash
uv run llm-chat --voice --whisper-device mps
```

You can override the speech model or language hint:

```bash
uv run llm-chat --voice --whisper-model base.en --voice-language en
```

You can tune `whisper-stream` responsiveness:

```bash
uv run llm-chat --voice --whisper-stream-step 500 --whisper-stream-length 5000
```

You can tune Python Whisper live transcription responsiveness:

```bash
uv run llm-chat --voice --voice-engine python --realtime-interval 0.5 --realtime-window 5.0 --commit-interval 5.0 --keep-overlap 0.2
```

On macOS, microphone access may require Microphone permission for the terminal app. The first voice run may also download the selected Whisper model weights.

## Project Layout

```text
src/llm_cli_chat/
  agent.py      # fast-agent setup and lifecycle wrapper
  app.py        # Textual TUI
  cli.py        # console entrypoint
  voice.py      # Space-toggle microphone capture and Whisper transcription
  __main__.py   # python -m llm_cli_chat
```
