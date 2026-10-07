#!/usr/bin/env python3
"""
Run this ON THE LAPTOP (not the Jetson) to confirm rosbridge is reachable
over Wi-Fi and is relaying real MAVROS telemetry.

Requires: pip install roslibpy

Usage:
    python3 laptop_rosbridge_smoke_test.py [jetson_ip]

Defaults to the Jetson's current Wi-Fi IP (10.145.63.112) if not given.
"""

import sys
import time

import roslibpy

JETSON_IP = sys.argv[1] if len(sys.argv) > 1 else "10.145.63.112"
PORT = 9090

TOPICS = [
    ("/mavros/imu/data", "sensor_msgs/Imu"),
    ("/mavros/battery", "sensor_msgs/BatteryState"),
    ("/mavros/state", "mavros_msgs/State"),
]


def main():
    print(f"Connecting to ws://{JETSON_IP}:{PORT} ...")
    client = roslibpy.Ros(host=JETSON_IP, port=PORT)
    client.run(timeout=5)

    if not client.is_connected:
        print("FAILED to connect. Check Wi-Fi, IP, and that rosbridge is running on the Jetson.")
        sys.exit(1)

    print("Connected.\n")

    received = {topic: 0 for topic, _ in TOPICS}
    listeners = []

    def make_cb(topic):
        def cb(msg):
            received[topic] += 1
            if received[topic] == 1:
                print(f"[{topic}] first message received:")
                print(f"  {msg}\n")
        return cb

    for topic, msg_type in TOPICS:
        listener = roslibpy.Topic(client, topic, msg_type)
        listener.subscribe(make_cb(topic))
        listeners.append(listener)

    print("Listening for 8 seconds...\n")
    time.sleep(8)

    for listener in listeners:
        listener.unsubscribe()
    client.terminate()

    print("=== Summary ===")
    ok = True
    for topic, _ in TOPICS:
        count = received[topic]
        status = "OK" if count > 0 else "NO DATA"
        if count == 0:
            ok = False
        print(f"  {topic}: {count} messages ({status})")

    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
