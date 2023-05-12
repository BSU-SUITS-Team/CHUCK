from fastapi import APIRouter, WebSocket, WebSocketDisconnect

router = APIRouter(prefix="/ws", tags=["ws"])


class WebSocketManager:
    def __init__(self):
        self.websockets = []

    async def connect(self, ws):
        await ws.accept()
        self.websockets.append(ws)

    def disconnect(self, ws):
        self.websockets.remove(ws)

    async def broadcast(self, ws, data):
        for ws_conn in self.websockets:
            if ws != ws_conn:
                await ws_conn.send_text(data)


ws_manager = WebSocketManager()


@router.websocket("/updates")
async def connect_to_updates(websocket: WebSocket):
    await ws_manager.connect(websocket)
    try:
        while True:
            data = await websocket.receive_text()
            await ws_manager.broadcast(websocket, data)
    except WebSocketDisconnect:
        ws_manager.disconnect(websocket)
