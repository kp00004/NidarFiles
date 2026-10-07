"""FastAPI GCS backend for NIDAR AirMouse.

Owns the single connection to rosbridge (the real Jetson Nano, or
sim/rosbridge_sim during development) and exposes it to the frontend as a
plain REST API. The entire operator command surface is exactly two
routes: POST /api/command/start and POST /api/command/abort — see
docs/REQUIREMENTS.md §4/§6 and docs/DECISIONS.md D-0.
"""
