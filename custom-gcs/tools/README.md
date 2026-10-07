# tools/

Development and field tooling that isn't part of the GCS application itself.

- `laptop_rosbridge_smoke_test.py` — checks the laptop can reach the
  Jetson's rosbridge over Wi-Fi.
- `telem_command.py` — terminal tool to send START (Hover) / ABORT over the
  MicroLR900 command radio, using the backend's own `RadioLink`. See
  `docs/COMMUNICATION.md` §6.

## telem_command.py

Needs the Jetson running `radio_command_node` (onboard-autonomy). For a
radio-only test, run it with `-p dry_run:=true`: the Jetson ACKs and logs
but publishes nothing, so nothing can arm.

```powershell
pip install -r gcs/backend/requirements.txt
python tools/telem_command.py --port COM5
```

At the `>` prompt: `status`, `start` (asks you to type START), `abort`,
`quit`. Confirmation comes only from the Jetson's ACK and heartbeat.
