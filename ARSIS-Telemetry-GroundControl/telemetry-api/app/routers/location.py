from fastapi import APIRouter, Request, status, Response, Depends
from sqlalchemy.orm import Session
from ..db.database import get_db
from ..db.schemas import LocationBase
from .. import crud

router = APIRouter(prefix="/location", tags=["location"])

@router.get("/")
async def location(db: Session = Depends(get_db)):
    all_users = crud.get_users(db)
    users_list = []
    for user in all_users:
        users_list.append({ "name": user["name"], "location": user["location"] })
    data = {"users": users_list}

    return data


@router.get("/{user}")
async def user_location(res: Response, user_id: int, db: Session = Depends(get_db)):
    user_data = crud.get_user(db, user_id)
    if not user_data:
        res.status_code = status.HTTP_404_NOT_FOUND
        return {"error": f"User with id: {user_id} not found"}
    return user_data["location"]


@router.post("/{user}/update_location")
async def update_user_location(res: Response, user_id: int, new_location: LocationBase, db: Session = Depends(get_db)):
    user_data = crud.update_user_location(db, user_id, new_location)
    if user_data is None:
        res.status_code = status.HTTP_400_BAD_REQUEST
        return {"error": f"User with id: {user_id} not found"}
    return { "message": f"Successfully updated location for {user_id}"}