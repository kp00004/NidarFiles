"""Bench/debug tool: send START (Hover) or ABORT over the MicroLR900 command
radio from a terminal, without the GCS UI. Uses the GCS backend's own
RadioLink (gcs/backend/app/radio_link.py), so it exercises exactly the
code the GCS uses.

    Laptop --USB--> MicroLR900 ))) MicroLR900 --USB--> Jetson radio_command_node

Confirmation comes only from the Jetson (its COMMAND_ACK and its radio
heartbeat). The Pixhawk is not on the radio link.

Usage (Windows, from the custom-gcs folder):
    pip install -r gcs/backend/requirements.txt
    python tools/telem_command.py --port COM5

Prompt commands: status | start | abort | quit
For a radio-only bench test, run the Jetson node with -p dry_run:=true:
the Jetson ACKs and logs but publishes nothing, so nothing can arm.
"""

from __future__ import annotations

import argparse
import logging
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "gcs", "backend"))

from app.missions import get_mission  # noqa: E402
from app.radio_link import RadioLink, RadioUnavailable  # noqa: E402


def _print_status(link: RadioLink) -> None:
    s = link.status()
    if not s["port_open"]:
        print(f"  radio port {s['port']} NOT OPEN: {s['port_error']}")
    elif not s["jetson_link_up"]:
        print("  RADIO LINK DOWN -- no heartbeat from the Jetson")
    else:
        print(
            f"  link UP, Jetson heartbeat {s['jetson_heartbeat_age_s']} s ago, "
            f"mission state: {s['jetson_mission_state']}"
        )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--port", default="COM5", help="radio COM port (default COM5)")
    parser.add_argument("--baud", type=int, default=115200)
    args = parser.parse_args()
    logging.basicConfig(level=logging.WARNING, format="%(asctime)s %(message)s")

    link = RadioLink(args.port, args.baud)
    link.start()
    print(f"Opening {args.port} @ {args.baud}; waiting for the Jetson heartbeat...")
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline and not link.status()["jetson_link_up"]:
        time.sleep(0.2)
    _print_status(link)
    print("Commands: status | start | abort | quit")

    try:
        while True:
            line = input("> ").strip().lower()
            if line in ("quit", "exit", "q"):
                break
            if line in ("", "status"):
                _print_status(link)
                continue
            if line not in ("start", "abort"):
                print("  unknown command -- use status | start | abort | quit")
                continue
            if line == "start":
                confirm = input("  START (Hover) can arm and fly the vehicle. Type START to confirm: ")
                if confirm.strip() != "START":
                    print("  cancelled")
                    continue
            code = get_mission("hover").radio_code if line == "start" else 0
            try:
                result = link.send_command(line, code)
            except RadioUnavailable as exc:
                print(f"  NOT SENT: {exc}")
                continue
            if not result.acked:
                print(f"  NO ACK from the Jetson after {result.attempts} attempts")
            elif result.accepted:
                print(f"  Jetson ACCEPTED {line.upper()} (attempts: {result.attempts})")
            else:
                print(f"  Jetson REJECTED {line.upper()}: {result.reason}")
    except (KeyboardInterrupt, EOFError):
        print()
    finally:
        link.stop()


if __name__ == "__main__":
    main()
