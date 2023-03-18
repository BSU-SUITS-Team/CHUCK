from fastapi import APIRouter, Request, status, Response, Depends
from sqlalchemy.orm import Session
from ..db.database import get_db
from .. import crud

router = APIRouter(prefix="/user", tags=["user"])


@router.get("/")
def get_all_users(db: Session = Depends(get_db)):
    return crud.get_users(db)


@router.get("/{user}")
async def get_user(res: Response, user_id: int, db: Session = Depends(get_db)):
    user = crud.get_user(db=db, user_id=user_id)
    if user is None:
        res.status_code = status.HTTP_404_NOT_FOUND
        return {"error": f"User with id: {user_id} not found"}
    return user


@router.put("/{user}", status_code=status.HTTP_201_CREATED)
async def put_user(username: str, db: Session = Depends(get_db)):
    user_id = crud.register_user(db=db, user=username).uuid
    
    init_bios = { "o2": 0, "battery": 0, "bpm": 0 }
    init_loc = { "latitude": 0, "longitude": 0, "altitude": 0, "heading": 0 }

    crud.update_user_biometrics(db=db, user_id=user_id, biometrics=init_bios)
    crud.update_user_location(db=db, user_id=user_id, location=init_loc)
    
    return { "user_id": user_id }