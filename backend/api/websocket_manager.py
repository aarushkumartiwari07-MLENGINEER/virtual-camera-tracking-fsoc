"""
WebSocket Connection Manager for real-time telemetry streaming and event broadcasting.
Maintains active client connections and provides safe asynchronous broadcasting.
"""

import json
import logging
from typing import List, Dict, Any, Optional
from fastapi import WebSocket, WebSocketDisconnect

logger = logging.getLogger("api.websocket")


class WebSocketManager:
    """
    Manages active WebSocket client connections and broadcasts structured JSON messages.
    """

    def __init__(self) -> None:
        self.active_connections: List[WebSocket] = []

    async def connect(self, websocket: WebSocket) -> None:
        """Accept connection and register client."""
        await websocket.accept()
        self.active_connections.append(websocket)
        logger.info(f"WebSocket client connected. Total active: {len(self.active_connections)}")

    def disconnect(self, websocket: WebSocket) -> None:
        """Unregister client on disconnect."""
        if websocket in self.active_connections:
            self.active_connections.remove(websocket)
            logger.info(f"WebSocket client disconnected. Total active: {len(self.active_connections)}")

    async def broadcast_json(self, message: Dict[str, Any]) -> None:
        """
        Broadcast a structured JSON message to all connected clients.
        Automatically removes stale/disconnected clients.
        """
        if not self.active_connections:
            return

        dead_connections: List[WebSocket] = []
        for connection in self.active_connections:
            try:
                await connection.send_json(message)
            except Exception as e:
                logger.warning(f"Failed to send to WebSocket client: {e}")
                dead_connections.append(connection)

        for dead in dead_connections:
            self.disconnect(dead)

    async def broadcast_event(self, category: str, message: str, level: str = "INFO", data: Optional[Dict[str, Any]] = None) -> None:
        """Broadcast a timestamped system event log."""
        import time
        event_payload = {
            "type": "event_log",
            "timestamp": time.time(),
            "category": category,
            "level": level,
            "message": message,
            "data": data or {},
        }
        await self.broadcast_json(event_payload)

    async def broadcast_telemetry(self, telemetry: Dict[str, Any]) -> None:
        """Broadcast a live simulation and tracking telemetry frame."""
        await self.broadcast_json({
            "type": "telemetry",
            "payload": telemetry,
        })

    async def broadcast_state_change(self, simulation_status: str, tracking_state: str) -> None:
        """Broadcast simulation or tracking state transitions."""
        await self.broadcast_json({
            "type": "state_change",
            "simulation_status": simulation_status,
            "tracking_state": tracking_state,
        })


# Global singleton instance
ws_manager = WebSocketManager()
