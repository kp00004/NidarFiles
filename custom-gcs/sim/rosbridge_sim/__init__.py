"""Rosbridge-protocol-compatible simulator for the NIDAR AirMouse drone side.

Speaks the same WebSocket wire protocol as the real Jetson Nano's
`rosbridge_server` (see docs/COMMUNICATION.md), publishing synthetic data
matching docs/DATA_MODELS.md, so the GCS frontend and other clients can be
developed and tested against a stand-in for the real drone.
"""
