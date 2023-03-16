from fastapi import APIRouter, Request, status, Depends, Response
from sqlalchemy.orm import Session
from ..db.database import get_db

router = APIRouter(prefix="/user", tags=["user"])


@router.get("/")
def get_all_users(request: Request):
    return request.app.user_cache.get_all()


@router.get("/{user}")
async def get_user(request: Request, res: Response, user: str, db: Session = Depends(get_db)):
    user_data = request.app.user_cache.get(user, db)
    if user_data is None:
        res.status_code = status.HTTP_404_NOT_FOUND
        return {"error": f"User {user} not found"}
    return user_data


@router.put("/{user}", status_code=status.HTTP_201_CREATED)
async def put_user(req: Request, res: Response, user: str, db: Session = Depends(get_db)):
    user_id = req.app.user_cache.register(user, db)
    if user_id is None:
        res.status_code = status.HTTP_409_CONFLICT
        return {"error": f"User {user} already exists"}
    return { "message": f"User {user} registered" }
