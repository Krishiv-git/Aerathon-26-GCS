# Phoenix GCS frontend

Run the Next.js flood mission control panel from this directory:

```bash
npm install
npm run dev -- --hostname 127.0.0.1
```

Open `http://127.0.0.1:3000`. Start the backend first using the command in the repository README. The app opens on the full globe; the bundled `public/mock_arena_geofence.kml` is the default mission area. Use **UPLOAD** to choose another KML and fly the globe to its bounds.

## Deploy

Deploy this directory as the Vercel project root. Set `NEXT_PUBLIC_BACKEND_URL` to the public URL of the separately hosted backend. The backend runs a persistent PyBullet simulation worker and is provided as a container in `backend/Dockerfile` (build with the repository root as the Docker context). The bundled KML is copied into that image.
