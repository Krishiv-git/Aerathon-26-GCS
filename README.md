# Aerathon 26 GCS — Phoenix Flood Mission Simulation

This repository runs one local, simulated flood rescue mission. It opens on the full Cesium globe, with `mock_arena_geofence.kml` as the default mission area and a PyBullet flood world.

## Start the application

Open two terminals from the repository root.

Terminal 1 — backend:

```bash
cd backend
./venv/bin/uvicorn main:app --reload --host 127.0.0.1 --port 8000
```

Terminal 2 — frontend:

```bash
cd frontend
npm run dev -- --hostname 127.0.0.1
```

Open [http://127.0.0.1:3000](http://127.0.0.1:3000). Use the small **UPLOAD** button to choose a KML and animate the globe to its mission area, or use the bundled KML. Select **START FLOOD MISSION** to run the same PyBullet route at 5 m and 3 m/s. The backend health endpoint is [http://127.0.0.1:8000/health](http://127.0.0.1:8000/health).

## Setup on a fresh install

```bash
cd backend
python3 -m venv venv
./venv/bin/pip install -r requirements.txt
cd ../frontend
npm install
```

The globe's satellite imagery is served by Esri and CesiumJS is loaded from its CDN, so those map layers need an internet connection.

This project is a simulator demo. It does not connect to aircraft or control real flight hardware.
