# Flood simulation backend

The FastAPI app serves only the simulated flood mission. It uses an uploaded KML when provided, otherwise it loads `frontend/public/mock_arena_geofence.kml`, and runs the PyBullet backend with the UI's altitude and speed settings.

From the repository root:

```bash
cd backend
./venv/bin/uvicorn main:app --reload --host 127.0.0.1 --port 8000
```

Check the process at `GET /health`; the frontend uses `POST /simulation/flood/start`, `GET /simulation/flood/{session_id}/state`, the camera endpoints, and `POST /simulation/flood/{session_id}/stop`.

For a persistent container deployment, build `backend/Dockerfile` using the repository root as the build context. Configure the frontend's `NEXT_PUBLIC_BACKEND_URL` with the backend's public base URL.
