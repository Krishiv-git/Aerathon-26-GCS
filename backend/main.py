"""FastAPI entry point for the simulated flood mission."""

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from flood_sim_api import router as flood_sim_router

app = FastAPI(title="Phoenix GCS Flood Simulation", version="1.0.0")
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)
app.include_router(flood_sim_router)


@app.get("/health")
def health():
    return {"status": "ok", "mission": "simulated-flood"}


if __name__ == "__main__":
    import uvicorn

    uvicorn.run("main:app", host="127.0.0.1", port=8000, reload=True)
