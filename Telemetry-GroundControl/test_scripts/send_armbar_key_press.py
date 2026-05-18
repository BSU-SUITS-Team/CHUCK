#!/usr/bin/env python3
import argparse
import os
import sys

import requests


DEFAULT_API_URL = os.environ.get("GROUNDCONTROL_API_URL", "http://localhost:8181")


def build_parser():
    parser = argparse.ArgumentParser(
        description="Send a debug armbar button press event to groundcontrol-api."
    )
    parser.add_argument(
        "key",
        help="Armbar button to press. Accepts 1-6, B1-B6, or button1-button6.",
    )
    parser.add_argument(
        "--api",
        default=DEFAULT_API_URL,
        help=f"Base groundcontrol-api URL. Defaults to {DEFAULT_API_URL}.",
    )
    parser.add_argument("--source", default="debug_script")
    parser.add_argument("--target", default="hololens")
    return parser


def main():
    args = build_parser().parse_args()
    url = f"{args.api.rstrip('/')}/armbar/key-presses"
    payload = {
        "key": args.key,
        "source": args.source,
        "target": args.target,
    }

    try:
        response = requests.post(url, json=payload, timeout=5)
        response.raise_for_status()
    except requests.RequestException as exc:
        print(f"Failed to send armbar key press event: {exc}", file=sys.stderr)
        return 1

    print(response.json())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
