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

The default model is the llama.cpp server model exposed at
`http://127.0.0.1:8012/v1` via fast-agent's `generic` OpenAI-compatible
provider. Start `llama-server` with the model you want to use:

```bash
llama-server -hf ggml-org/Qwen3-Coder-30B-A3B-Instruct-Q8_0-GGUF --host 127.0.0.1 --port 8012
```

The configured default model id is:

```bash
ggml-org/Qwen3-Coder-30B-A3B-Instruct-Q8_0-GGUF
```

If you start `llama-server` with `--alias`, use that alias in `fastagent.config.yaml`
and in `LLM_CHAT_MODEL`. You can confirm the active model id with:

```bash
curl http://127.0.0.1:8012/v1/models
```

You can override the default llama.cpp endpoint if needed:

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
uv run llm-chat --model "generic.ggml-org/Qwen3-Coder-30B-A3B-Instruct-Q8_0-GGUF"
```

Inside the app, type a message and press Enter. Use `Ctrl+C` to quit and `Ctrl+L` to clear the chat log.
Assistant responses stream into the chat log as model chunks arrive.

Before each typed or voice prompt is sent to the model, the app fetches current mission context and prepends it to the model-visible message:

- current EVA biometrics from `TSS_ENDPOINT` or `http://localhost:14141`
- available procedure names from `GROUND_CONTROL_API_URL` or `http://localhost:8181`

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
