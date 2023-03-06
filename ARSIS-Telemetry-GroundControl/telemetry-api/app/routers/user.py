from fastapi import APIRouter, Request
import string
import random

router = APIRouter(prefix="/user", tags=["user"])

@router.get("/")
def get_all_users(request: Request):
    return request.app.user_cache.get_all()

@router.get("/{user}")
async def get_user(request: Request, user: str):

    return request.app.user_cache.get(user)


@router.put("/{user}")
async def put_user(request: Request, user: str):
    request.app.user_cache.register(user)
