from fastapi import APIRouter

router = APIRouter(prefix="/procedures", tags=["procedures"])

@router.get("/")
async def procedures():
    return None


@router.get("/{name}")
def procedure(name: str):
    return {"name": "Not found", "taskList": []}

