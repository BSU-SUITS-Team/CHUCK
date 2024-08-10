from fastapi import APIRouter

router = APIRouter(prefix="/webrtc", tags=["webrtc"])

@router.get("/")
async def webrtc():
    return None


@router.get("/{name}")
def webrtc(name: str):
    return {"name": "Not found", "taskList": []}

