"""CLI entrypoint: `python -m rosbridge_sim [options]`.

Runs the simulator as a standalone rosbridge-protocol server, exactly as
the real Jetson's `rosbridge_server` would present itself to a client.
"""

from __future__ import annotations

import argparse
import asyncio

from .mission import MissionConfig, MissionSimulator
from .server import SimServer


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="NIDAR AirMouse rosbridge simulator")
    parser.add_argument("--host", default="0.0.0.0", help="bind host (default: 0.0.0.0)")
    parser.add_argument("--port", type=int, default=9090, help="bind port (default: 9090, matches rosbridge_server)")
    parser.add_argument("--width", type=int, default=15, help="arena width in cells (default: 15, rulebook max)")
    parser.add_argument("--height", type=int, default=15, help="arena height in cells (default: 15, rulebook max)")
    parser.add_argument("--resolution", type=float, default=1.0, help="meters per grid cell (default: 1.0)")
    parser.add_argument("--duration", type=float, default=300.0, help="simulated mission duration in seconds (default: 300; rulebook max is 1800)")
    parser.add_argument("--survivors", type=int, default=6, help="number of survivors to place (default: 6, rulebook max)")
    parser.add_argument("--seed", type=int, default=42, help="RNG seed for reproducible arenas/survivors (default: 42)")
    return parser.parse_args()


def main() -> None:
    args = _parse_args()
    config = MissionConfig(
        width=args.width,
        height=args.height,
        resolution=args.resolution,
        duration_s=args.duration,
        num_survivors=args.survivors,
        seed=args.seed,
    )
    mission = MissionSimulator(config)
    server = SimServer(mission, host=args.host, port=args.port)

    print(f"rosbridge_sim listening on ws://{args.host}:{args.port}", flush=True)
    print(f"arena {config.width}x{config.height} @ {config.resolution} m/cell, "
          f"{config.num_survivors} survivors, {config.duration_s:.0f}s mission, seed={config.seed}", flush=True)
    print('publish {"op":"publish","topic":"/gcs/command","msg":{"data":"start"}} to begin the mission', flush=True)

    try:
        asyncio.run(server.run_forever())
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
