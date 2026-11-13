# Flood mission runtime

The browser mission backend uses these modules:

- `flood_mission.py` runs the PyBullet survey and publishes state and camera frames.
- `flood_world.py` builds the synthetic flooded arena, obstacles, and people.
- `flood_support.py` contains the kinematic vehicle, drone model, and downward camera adapter.

Start the application from the repository root using the backend and frontend commands in `README.md`. The FastAPI bridge loads the fixed KML from `frontend/public/mock_arena_geofence.kml` and sends simulation output to a temporary directory.
