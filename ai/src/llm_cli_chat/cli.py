from __future__ import annotations

import argparse
import os
from pathlib import Path

from llm_cli_chat.agent import DEFAULT_MODEL, FastAgentChatBackend
from llm_cli_chat.app import ChatApp
from llm_cli_chat.context import (
    DEFAULT_CONTEXT_TIMEOUT_SECONDS,
    DEFAULT_GROUND_CONTROL_API_URL,
    DEFAULT_TSS_ENDPOINT,
    MissionContextProvider,
    USER_EVA_CHOICES,
    normalize_user_eva,
)
from llm_cli_chat.voice import VoiceInputConfig, load_whisper_model, resolve_whisper_device


DEFAULT_WHISPER_MODEL = "tiny.en"


def parse_user_eva(value: str) -> str:
    normalized = normalize_user_eva(value)
    if normalized is None:
        expected = ", ".join(USER_EVA_CHOICES)
        raise argparse.ArgumentTypeError(f"expected one of: {expected}")
    return normalized


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run the LLM chat TUI.")
    parser.add_argument(
        "--model",
        default=os.getenv("LLM_CHAT_MODEL", DEFAULT_MODEL),
        help=(
            "Model string passed to fast-agent "
            f"(default: {os.getenv('LLM_CHAT_MODEL', DEFAULT_MODEL)!r})."
        ),
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=None,
        help="Optional path to a fastagent.config.yaml file.",
    )
    parser.add_argument(
        "--voice",
        action="store_true",
        help="Enable Space-toggle voice input instead of the text input box.",
    )
    parser.add_argument(
        "--user-eva",
        type=parse_user_eva,
        default=normalize_user_eva(os.getenv("LLM_CHAT_USER_EVA")),
        metavar="{eva1,eva2}",
        help="Specify whether the user is EVA 1 or EVA 2 for first-person context.",
    )
    parser.add_argument(
        "--no-mission-context",
        action="store_true",
        help="Send prompts without current biometrics and available procedures context.",
    )
    parser.add_argument(
        "--ground-control-api-url",
        default=os.getenv("GROUND_CONTROL_API_URL", DEFAULT_GROUND_CONTROL_API_URL),
        help=(
            "Ground Control API base URL used to fetch available procedures "
            f"(default: {os.getenv('GROUND_CONTROL_API_URL', DEFAULT_GROUND_CONTROL_API_URL)!r})."
        ),
    )
    parser.add_argument(
        "--tss-endpoint",
        default=os.getenv("TSS_ENDPOINT", DEFAULT_TSS_ENDPOINT),
        help=(
            "TSS base URL used to fetch current EVA biometrics "
            f"(default: {os.getenv('TSS_ENDPOINT', DEFAULT_TSS_ENDPOINT)!r})."
        ),
    )
    parser.add_argument(
        "--context-timeout",
        type=float,
        default=float(os.getenv("LLM_CHAT_CONTEXT_TIMEOUT", DEFAULT_CONTEXT_TIMEOUT_SECONDS)),
        help="Seconds to wait for each mission context source before sending the prompt.",
    )
    parser.add_argument(
        "--whisper-model",
        default=os.getenv("LLM_CHAT_WHISPER_MODEL", DEFAULT_WHISPER_MODEL),
        help=(
            "Whisper speech recognition model used with --voice "
            f"(default: {os.getenv('LLM_CHAT_WHISPER_MODEL', DEFAULT_WHISPER_MODEL)!r})."
        ),
    )
    parser.add_argument(
        "--whisper-device",
        choices=("auto", "cpu", "cuda", "mps"),
        default=os.getenv("LLM_CHAT_WHISPER_DEVICE", "auto"),
        help="Device for Whisper. auto uses CUDA when available, otherwise CPU.",
    )
    parser.add_argument(
        "--voice-language",
        default=os.getenv("LLM_CHAT_WHISPER_LANGUAGE"),
        help="Optional spoken language hint for Whisper, for example 'en'.",
    )
    parser.add_argument(
        "--realtime-interval",
        type=float,
        default=float(os.getenv("LLM_CHAT_REALTIME_INTERVAL", "0.25")),
        help="Seconds between live transcription updates in voice mode.",
    )
    parser.add_argument(
        "--realtime-window",
        type=float,
        default=float(os.getenv("LLM_CHAT_REALTIME_WINDOW", "2.0")),
        help="Seconds of recent audio to use for each live transcription update.",
    )
    parser.add_argument(
        "--commit-interval",
        type=float,
        default=float(os.getenv("LLM_CHAT_COMMIT_INTERVAL", "1.5")),
        help="Seconds of new audio to commit between live transcript drafts.",
    )
    parser.add_argument(
        "--keep-overlap",
        type=float,
        default=float(os.getenv("LLM_CHAT_KEEP_OVERLAP", "0.2")),
        help="Seconds of prior audio to keep at chunk boundaries.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    context_provider = None
    if not args.no_mission_context:
        context_provider = MissionContextProvider(
            ground_control_api_url=args.ground_control_api_url,
            tss_endpoint=args.tss_endpoint,
            timeout_seconds=args.context_timeout,
            user_eva=args.user_eva,
        )

    backend = FastAgentChatBackend(
        model=args.model,
        config_path=args.config,
        context_provider=context_provider,
        include_mission_context=not args.no_mission_context,
    )
    voice_config = VoiceInputConfig(
        whisper_model=args.whisper_model,
        whisper_device=args.whisper_device,
        realtime_interval_seconds=args.realtime_interval,
        realtime_window_seconds=args.realtime_window,
        commit_interval_seconds=args.commit_interval,
        keep_overlap_seconds=args.keep_overlap,
        language=args.voice_language,
    )
    voice_model = None
    if args.voice:
        whisper_device = resolve_whisper_device(args.whisper_device)
        print(f"Loading Whisper model '{args.whisper_model}' on {whisper_device}...")
        try:
            voice_model = load_whisper_model(args.whisper_model, device=args.whisper_device)
        except Exception as exc:  # noqa: BLE001 - fail before Textual takes over the terminal.
            raise SystemExit(f"Could not load Whisper model '{args.whisper_model}': {exc}") from exc

    ChatApp(
        backend,
        voice_enabled=args.voice,
        voice_config=voice_config,
        voice_model=voice_model,
    ).run()
