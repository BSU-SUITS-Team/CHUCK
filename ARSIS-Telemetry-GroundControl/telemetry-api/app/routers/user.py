from fastapi import APIRouter, Request, status
import string
import random

router = APIRouter(prefix="/user", tags=["user"])


@router.get("/")
def get_all_users(request: Request):
    return request.app.user_cache.get_all()


@router.get("/{user}")
async def get_user(request: Request, user: str):

    to_return = request.app.user_cache.get(user)
    to_return = to_return if to_return is not None else status.HTTP_404_NOT_FOUND
    return to_return


@router.put("/{user}")
async def put_user(request: Request, user: str):
    request.app.user_cache.register(user)
