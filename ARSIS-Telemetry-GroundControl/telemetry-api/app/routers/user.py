from fastapi import APIRouter, Request, status, Response

router = APIRouter(prefix="/user", tags=["user"])


@router.get("/")
def get_all_users(request: Request):
    return request.app.user_cache.get_all()


@router.get("/{user}")
async def get_user(request: Request, res: Response, user_id: int):
    user_data = request.app.user_cache.get(user_id)
    if user_data is None:
        res.status_code = status.HTTP_404_NOT_FOUND
        return {"error": f"User with id: {user_id} not found"}
    return user_data


@router.put("/{user}", status_code=status.HTTP_201_CREATED)
async def put_user(req: Request, username: str):
    user_id = req.app.user_cache.register(username)
    return { "user_id": user_id }