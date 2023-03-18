from fastapi import APIRouter, Request
from pydantic import BaseModel, Field


router = APIRouter(prefix="/uia", tags=["uia"])

class UIAStatusResponse(BaseModel):
    o2: bool
    power: bool
    comm: bool

uia_status = {
    "o2": True,
    "power": True,
    "comm": True
}

@router.get("/")
async def get_uia() -> UIAStatusResponse:
    return uia_status

@router.post("/")
async def post_uia() -> UIAStatusResponse:
    return uia_status