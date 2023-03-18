from fastapi import APIRouter, status, Response, Depends
from sqlalchemy.orm import Session
from ..db.database import get_db
from .. import crud
from ..db import schemas

router = APIRouter(
    prefix="/biometrics",
    tags=["biometrics"]
)

@router.get("/")
async def get_biometrics(db: Session = Depends(get_db)):
    all_users = crud.get_users(db)
    users_list = []
    for user in all_users:
        users_list.append({ "name": user["name"], "biometrics": user["biometrics"] })
    data = {"users": users_list}
    return data

@router.get("/{user}")
async def user_biometrics(res: Response, user_id: int, db: Session = Depends(get_db)):
    user_info = crud.get_user(db, user_id)
    if not user_info:
        res.status_code = status.HTTP_404_NOT_FOUND
        return {"error": f"User with id: {user_id} not found"}
    return user_info["biometrics"]

@router.post("/{user}/update_biometrics")
async def update_user_biometrics(res: Response, user_id: int, new_biometrics: schemas.BiometricsBase, db: Session = Depends(get_db)):
    user_data = crud.update_user_biometrics(db, user_id, new_biometrics)
    if user_data is None:
        res.status_code = status.HTTP_400_BAD_REQUEST
        return {"error": f"User with id: {user_id} not found"}
    return { "message": f"Successfully updated biometrics for {user_id}"}