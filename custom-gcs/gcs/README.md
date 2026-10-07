# gcs/

The GCS application: Presentation Layer (frontend) + Link/Bridge Layer
(backend) — see `../docs/ARCHITECTURE.md`.

- `backend/` — **built.** A FastAPI service, the only component that
  talks to rosbridge (real Jetson or `../sim/`). Also serves `frontend/`
  as static files at `/ui`. See `backend/README.md`.
- `frontend/` — **built (minimal).** A single static HTML/JS page (no
  build step, no framework), calling the backend's REST API only. Exactly
  two controls (START, STOP/ABORT) plus a read-only telemetry dashboard —
  see `frontend/index.html`.
