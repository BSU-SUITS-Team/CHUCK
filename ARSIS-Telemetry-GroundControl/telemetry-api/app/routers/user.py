from fastapi import APIRouter, Request, status, Response

router = APIRouter(prefix="/user", tags=["user"])


@router.get("/")
def get_all_users(request: Request):
    return request.app.user_cache.get_all()


@router.get("/{user}")
async def get_user(request: Request, res: Response, user: str):
    user_data = request.app.user_cache.get(user)
    if user_data is None:
        res.status_code = status.HTTP_404_NOT_FOUND
        return {"error": f"User {user} not found"}
    return user_data


@router.put("/{user}", status_code=status.HTTP_201_CREATED)
async def put_user(req: Request, user: str):
    req.app.user_cache.register(user)
    return { "message": f"User {user} registered" }