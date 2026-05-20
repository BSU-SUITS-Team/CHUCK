#!/usr/bin/env python3
import argparse
import logging
import os
import queue
import signal
import subprocess
import sys
import threading
import time
from dataclasses import dataclass

try:
    import requests
except ImportError:  # pragma: no cover - only exercised on machines without requests.
    requests = None

try:
    from gpiozero import Button
except ImportError:  # pragma: no cover - only exercised on machines without gpiozero.
    Button = None


API_URL_ENV = "GROUNDCONTROL_API_URL"
REMOTE_IP_ENV = "GROUNDCONTROL_REMOTE_IP"
DEFAULT_API_URL = "http://localhost:8181"
DEFAULT_CONFIG_FILE = os.environ.get("ARMBAR_GPIO_CONFIG", "/etc/default/armbar-gpio")
DEFAULT_PIN_MAP = "1:21,2:19,3:20,4:13,5:12,6:6,7:16,8:5"
DEFAULT_REMOTE_PORT = 8181
DEFAULT_QUEUE_TIMEOUT_SECONDS = 0.25
DEFAULT_SERVICE_NAME = os.environ.get("ARMBAR_GPIO_SERVICE", "armbar-gpio.service")


@dataclass(frozen=True)
class ButtonBinding:
    key: str
    gpio_pin: int


@dataclass(frozen=True)
class ButtonPress:
    binding: ButtonBinding
    pressed_at: float


def parse_pin_map(value):
    bindings = []
    seen_keys = set()
    seen_pins = set()

    for item in value.split(","):
        item = item.strip()
        if not item:
            continue

        try:
            key, gpio_pin = item.split(":", 1)
        except ValueError as exc:
            raise argparse.ArgumentTypeError(
                f"Invalid pin mapping {item!r}; expected KEY:GPIO"
            ) from exc

        key = key.strip()
        if not key:
            raise argparse.ArgumentTypeError(
                f"Invalid pin mapping {item!r}; key cannot be empty"
            )

        try:
            gpio_pin = int(gpio_pin.strip())
        except ValueError as exc:
            raise argparse.ArgumentTypeError(
                f"Invalid GPIO pin for {key!r}; expected a BCM pin number"
            ) from exc

        if gpio_pin < 0:
            raise argparse.ArgumentTypeError(
                f"Invalid GPIO pin for {key!r}; expected a BCM pin number"
            )
        if key in seen_keys:
            raise argparse.ArgumentTypeError(f"Duplicate armbar key mapping: {key}")
        if gpio_pin in seen_pins:
            raise argparse.ArgumentTypeError(f"Duplicate GPIO pin mapping: {gpio_pin}")

        seen_keys.add(key)
        seen_pins.add(gpio_pin)
        bindings.append(ButtonBinding(key=key, gpio_pin=gpio_pin))

    if not bindings:
        raise argparse.ArgumentTypeError("At least one KEY:GPIO mapping is required")

    return bindings


def read_config_file(config_file):
    config = {}

    try:
        with open(config_file, "r", encoding="utf-8") as file:
            lines = file.readlines()
    except FileNotFoundError:
        return config
    except OSError as exc:
        logging.warning("Could not read config file %s: %s", config_file, exc)
        return config

    for line in lines:
        line = line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue

        key, value = line.split("=", 1)
        config[key.strip()] = value.strip().strip("\"'")

    return config


def write_config_file(config_file, config):
    directory = os.path.dirname(os.path.abspath(config_file))
    if directory:
        os.makedirs(directory, exist_ok=True)

    temp_file = f"{config_file}.tmp"
    with open(temp_file, "w", encoding="utf-8") as file:
        file.write("# Written by armbar_gpio_server.py\n")
        for key in sorted(config):
            file.write(f"{key}={config[key]}\n")

    os.replace(temp_file, config_file)


def build_api_url(remote_ip, remote_port):
    remote_ip = remote_ip.strip()
    if remote_ip.startswith(("http://", "https://")):
        return remote_ip.rstrip("/")

    return f"http://{remote_ip}:{remote_port}"


def resolve_api_url(config_file, cli_api, remote_ip, remote_port):
    if cli_api:
        return cli_api.rstrip("/")
    if remote_ip:
        return build_api_url(remote_ip, remote_port)

    config = read_config_file(config_file)
    if config.get(API_URL_ENV):
        return config[API_URL_ENV].rstrip("/")
    if config.get(REMOTE_IP_ENV):
        return build_api_url(config[REMOTE_IP_ENV], remote_port)
    if os.environ.get(API_URL_ENV):
        return os.environ[API_URL_ENV].rstrip("/")
    if os.environ.get(REMOTE_IP_ENV):
        return build_api_url(os.environ[REMOTE_IP_ENV], remote_port)

    return DEFAULT_API_URL


def write_remote_config(config_file, remote_ip, remote_port):
    config = read_config_file(config_file)
    config[REMOTE_IP_ENV] = remote_ip.strip()
    config[API_URL_ENV] = build_api_url(remote_ip, remote_port)
    write_config_file(config_file, config)
    return config[API_URL_ENV]


def write_api_config(config_file, api_url):
    config = read_config_file(config_file)
    config[API_URL_ENV] = api_url.rstrip("/")
    write_config_file(config_file, config)
    return config[API_URL_ENV]


def is_running_under_systemd():
    return bool(os.environ.get("INVOCATION_ID") or os.environ.get("JOURNAL_STREAM"))


def is_service_active(service_name):
    try:
        result = subprocess.run(
            ["systemctl", "is-active", "--quiet", service_name],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )
    except FileNotFoundError:
        return False

    return result.returncode == 0


def follow_service_logs(service_name):
    command = ["journalctl", "-u", service_name, "-f", "-n", "50"]
    print(f"Following logs for {service_name}. Press Ctrl-C to detach.")

    try:
        return subprocess.run(command, check=False).returncode
    except KeyboardInterrupt:
        print()
        return 0
    except FileNotFoundError:
        print("journalctl was not found; cannot attach to service logs.", file=sys.stderr)
        return 1


def run_service_companion(args):
    if args.remote_ip:
        try:
            api_url = write_remote_config(
                args.config_file,
                args.remote_ip,
                args.remote_port,
            )
        except OSError as exc:
            print(
                f"Failed to write config file {args.config_file}: {exc}",
                file=sys.stderr,
            )
            return 1

        print(f"Wrote {args.config_file} with {API_URL_ENV}={api_url}")
    elif args.api:
        try:
            api_url = write_api_config(args.config_file, args.api)
        except OSError as exc:
            print(
                f"Failed to write config file {args.config_file}: {exc}",
                file=sys.stderr,
            )
            return 1

        print(f"Wrote {args.config_file} with {API_URL_ENV}={api_url}")
    else:
        print(
            f"{args.service_name} is already running; no remote IP was provided, "
            "so the config file was left unchanged."
        )

    if args.no_log_follow:
        print(f"{args.service_name} is still running.")
        return 0

    result = follow_service_logs(args.service_name)
    if is_service_active(args.service_name):
        print(f"{args.service_name} is still running.")
    else:
        print(f"{args.service_name} is no longer active.")

    return result


def build_parser():
    parser = argparse.ArgumentParser(
        description=(
            "Read armbar buttons from Raspberry Pi GPIO pins and emit "
            "armbar_button_press events through groundcontrol-api."
        )
    )
    parser.add_argument(
        "--api",
        default=None,
        help=(
            f"Base groundcontrol-api URL. Defaults to {DEFAULT_CONFIG_FILE}, "
            f"then {API_URL_ENV}, then {DEFAULT_API_URL}."
        ),
    )
    parser.add_argument(
        "--remote-ip",
        help=(
            "Remote groundcontrol-api host/IP. When the systemd service is already "
            "running, this is written to the config file before following logs."
        ),
    )
    parser.add_argument(
        "--remote-port",
        type=int,
        default=DEFAULT_REMOTE_PORT,
        help=f"Remote groundcontrol-api port used with --remote-ip. Defaults to {DEFAULT_REMOTE_PORT}.",
    )
    parser.add_argument(
        "--config-file",
        default=DEFAULT_CONFIG_FILE,
        help=f"Config file used for the service API target. Defaults to {DEFAULT_CONFIG_FILE}.",
    )
    parser.add_argument(
        "--service-name",
        default=DEFAULT_SERVICE_NAME,
        help=f"systemd service name to check and follow. Defaults to {DEFAULT_SERVICE_NAME}.",
    )
    parser.add_argument(
        "--no-log-follow",
        action="store_true",
        help="When the service is already running, update config and exit without following logs.",
    )
    parser.add_argument(
        "--force-foreground",
        action="store_true",
        help="Run a foreground GPIO listener even if the systemd service is already active.",
    )
    parser.add_argument(
        "--map",
        dest="pin_map",
        default=parse_pin_map(DEFAULT_PIN_MAP),
        type=parse_pin_map,
        help=(
            "Comma-separated armbar key to BCM GPIO map. Each button should be "
            f"wired between its GPIO pin and ground. Defaults to {DEFAULT_PIN_MAP}."
        ),
    )
    parser.add_argument(
        "--bounce-time",
        type=float,
        default=0.05,
        help="Software debounce window in seconds. Defaults to 0.05.",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=5.0,
        help="HTTP request timeout in seconds. Defaults to 5.",
    )
    parser.add_argument("--source", default="raspberry_pi_gpio")
    parser.add_argument("--target", default="hololens")
    parser.add_argument(
        "--log-level",
        default="INFO",
        choices=("DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"),
    )
    return parser


def create_button(binding, bounce_time):
    return Button(binding.gpio_pin, pull_up=True, bounce_time=bounce_time)


def build_press_handler(binding, press_queue):
    def handle_press():
        press_queue.put(ButtonPress(binding=binding, pressed_at=time.time()))

    return handle_press


def send_press(api_url, press, source, target, timeout):
    url = f"{api_url.rstrip('/')}/armbar/key-presses"
    payload = {
        "key": press.binding.key,
        "source": source,
        "target": target,
    }

    response = requests.post(url, json=payload, timeout=timeout)
    response.raise_for_status()
    return response


def event_sender(
    config_file,
    cli_api,
    remote_ip,
    remote_port,
    press_queue,
    stop_event,
    source,
    target,
    timeout,
):
    while not stop_event.is_set() or not press_queue.empty():
        try:
            press = press_queue.get(timeout=DEFAULT_QUEUE_TIMEOUT_SECONDS)
        except queue.Empty:
            continue

        try:
            api_url = resolve_api_url(config_file, cli_api, remote_ip, remote_port)
            response = send_press(api_url, press, source, target, timeout)
        except requests.RequestException as exc:
            logging.error(
                "Failed to send armbar key %s from GPIO %s: %s",
                press.binding.key,
                press.binding.gpio_pin,
                exc,
            )
        else:
            logging.info(
                "Sent armbar key %s from GPIO %s: HTTP %s",
                press.binding.key,
                press.binding.gpio_pin,
                response.status_code,
            )
        finally:
            press_queue.task_done()


def install_signal_handlers(stop_event):
    def request_stop(signum, _frame):
        logging.info("Received signal %s; shutting down", signum)
        stop_event.set()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)


def main():
    args = build_parser().parse_args()
    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format="%(asctime)s %(levelname)s %(message)s",
    )

    if args.remote_port <= 0:
        print("--remote-port must be greater than zero", file=sys.stderr)
        return 1

    if (
        is_service_active(args.service_name)
        and not is_running_under_systemd()
        and not args.force_foreground
    ):
        return run_service_companion(args)

    if Button is None:
        print(
            "gpiozero is required to read Raspberry Pi GPIO buttons. "
            "On Raspberry Pi OS, install it with: sudo apt install python3-gpiozero",
            file=sys.stderr,
        )
        return 1

    if requests is None:
        print(
            "requests is required to send events to groundcontrol-api. "
            "On Raspberry Pi OS, install it with: sudo apt install python3-requests",
            file=sys.stderr,
        )
        return 1

    if args.bounce_time < 0:
        print("--bounce-time must be zero or greater", file=sys.stderr)
        return 1
    if args.timeout <= 0:
        print("--timeout must be greater than zero", file=sys.stderr)
        return 1

    stop_event = threading.Event()
    press_queue = queue.Queue()
    install_signal_handlers(stop_event)

    buttons = []
    sender = threading.Thread(
        target=event_sender,
        args=(
            args.config_file,
            args.api,
            args.remote_ip,
            args.remote_port,
            press_queue,
            stop_event,
            args.source,
            args.target,
            args.timeout,
        ),
        daemon=True,
    )

    sender_started = False
    try:
        for binding in args.pin_map:
            button = create_button(binding, args.bounce_time)
            button.when_pressed = build_press_handler(binding, press_queue)
            buttons.append(button)
            logging.info(
                "Listening for armbar key %s on BCM GPIO %s",
                binding.key,
                binding.gpio_pin,
            )

        sender.start()
        sender_started = True
        logging.info(
            "Sending armbar events to %s",
            resolve_api_url(
                args.config_file,
                args.api,
                args.remote_ip,
                args.remote_port,
            ),
        )

        while not stop_event.is_set():
            time.sleep(0.5)
    finally:
        stop_event.set()
        if sender_started:
            press_queue.join()
        for button in buttons:
            button.close()
        if sender_started:
            sender.join(timeout=1)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
