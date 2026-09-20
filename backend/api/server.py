"""
Master FastAPI Application for SIH26169 FSOC Virtual Camera Tracking System.
Integrates REST API routes, WebSocket telemetry stream, and frontend static assets.
"""

import os
import logging
from contextlib import asynccontextmanager
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse

from .routes import (
    simulation_router,
    config_router,
    tracking_router,
    benchmark_router,
)
from .websocket_manager import ws_manager
from .services.simulation_service import sim_service

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(name)s: %(message)s")
logger = logging.getLogger("api.server")


@asynccontextmanager
async def lifespan(app: FastAPI):
    """Application startup and graceful shutdown lifecycle."""
    logger.info("Initializing SIH26169 Backend Service...")
    # Generate initial frame 0
    sim_service._execute_single_step()
    yield
    logger.info("Shutting down SIH26169 Backend Service...")
    await sim_service.stop()


app = FastAPI(
    title="SIH26169 FSOC Virtual Camera Tracking API",
    description="Backend API and Real-Time Telemetry Service for Coarse Alignment of Mobile FSOC Terminals (ISRO / DOS)",
    version="2.0.0",
    lifespan=lifespan,
)

# CORS Middleware for local web development
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include REST Routers
app.include_router(simulation_router)
app.include_router(config_router)
app.include_router(tracking_router)
app.include_router(benchmark_router)


# WebSocket Real-Time Telemetry Endpoint
@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    """
    Real-time bidirectional WebSocket channel for high-frequency telemetry,
    sensor frames, state changes, and interactive control commands.
    """
    await ws_manager.connect(websocket)
    try:
        # Send initial state snapshot upon connection
        status = sim_service.get_status()
        await websocket.send_json({
            "type": "connection_ack",
            "message": "Connected to SIH26169 Real-Time Telemetry Stream",
            "status": status,
        })
        if sim_service.last_telemetry_packet:
            await websocket.send_json({
                "type": "telemetry",
                "payload": sim_service.last_telemetry_packet,
            })

        while True:
            data = await websocket.receive_json()
            cmd = data.get("command")
            if cmd == "start":
                await sim_service.start()
            elif cmd == "pause":
                await sim_service.pause()
            elif cmd == "resume":
                await sim_service.resume()
            elif cmd == "stop":
                await sim_service.stop()
            elif cmd == "reset":
                motion = data.get("motion")
                await sim_service.reset(motion_type=motion)
            elif cmd == "step":
                await sim_service.step_frame()
            elif cmd == "ping":
                await websocket.send_json({"type": "pong"})

    except WebSocketDisconnect:
        ws_manager.disconnect(websocket)
    except Exception as e:
        logger.warning(f"WebSocket error: {e}")
        ws_manager.disconnect(websocket)


# Mount Frontend Static Assets
frontend_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "frontend"))
if os.path.isdir(frontend_dir):
    app.mount("/", StaticFiles(directory=frontend_dir, html=True), name="frontend")
else:
    @app.get("/")
    async def root_fallback():
        return {
            "name": "SIH26169 FSOC Virtual Camera Tracking API",
            "status": "online",
            "docs": "/docs",
        }


if __name__ == "__main__":
    import uvicorn
    uvicorn.run("backend.api.server:app", host="127.0.0.1", port=8000, reload=True)
