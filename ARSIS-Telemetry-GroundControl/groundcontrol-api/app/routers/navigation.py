from fastapi import APIRouter
from app.routers.on_server_navigation.default_paths import paths, paths_json


router = APIRouter(prefix="/navigation", tags=["navigation"])


@router.get("/")
async def procedures():
    return paths_json
