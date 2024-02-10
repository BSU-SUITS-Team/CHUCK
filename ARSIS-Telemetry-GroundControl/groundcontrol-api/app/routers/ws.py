from fastapi import APIRouter, WebSocket, WebSocketDisconnect
from app.datastore import ds

router = APIRouter(prefix="/ws", tags=["ws"])


class WebSocketManager:
    def __init__(self):
        self.websockets = {}

    async def connect(self, path, ws):
        await ws.accept()
        if path not in self.websockets:
            self.websockets[path] = []
        self.websockets[path].append(ws)

    def disconnect(self, path, ws):
        self.websockets[path].remove(ws)

    async def async_receive(self, path):
        pass

    async def broadcast(self, ws, data):
        for ws_conn in self.websockets:
            if ws != ws_conn:
                await ws_conn.send_text(data)

    async def broadcast_to_all(self, path, data):
        for ws_conn in self.websockets[path]:
            await ws_conn.send_json(data)


ws_manager = WebSocketManager()


@router.websocket("/events")
async def connect_to_events(websocket: WebSocket):
    await ws_manager.connect("events", websocket)
    ds_update_gen = ds.make_async_gen()
    try:
        all_data = await ds.get_all()
        for a in all_data:
            print(a)
            await websocket.send_json(a)
        while True:
            update = await ds_update_gen.__anext__()
            await ws_manager.broadcast_to_all("events", update)
    except WebSocketDisconnect:
        ws_manager.disconnect("events", websocket)
