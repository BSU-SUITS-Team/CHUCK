import string
import random

from fastapi import APIRouter, Request, status, Response, Depends
from pydantic import BaseModel
from sqlalchemy.orm import Session
from ..db.database import get_db

router = APIRouter(
    prefix="/biometrics",
    tags=["biometrics"]
)

random.seed(20031101)

class Biometrics(BaseModel):
    bpm: int
    o2: int
    battery: int

# bpm = 120
# o2 = 100
# battery = 100

# o2_drop_chance = 25
# battery_drop_chance = 25

# def evaluate_biometrics():
#     global bpm
#     global battery
#     global o2

#     bpm = random.randrange(95, 162)
#     if random.randint(0, 100) < battery_drop_chance and battery > 0:
#         battery -= random.randrange(0, 2)
#     elif battery <= 0:
#         battery = 0
#     if random.randint(0, 100) < o2_drop_chance and o2 > 0:
#         o2 -= random.randrange(0, 2)
#     elif o2 <= 0:
#         o2 = 0

# def create_random_string(characters = string.ascii_lowercase):
#     return ''.join(random.choice(characters) for _ in range(6))

@router.get("/")
async def get_biometrics(request: Request):
    all_users = request.app.user_cache.get_all()
    users_list = [{**{"user": k}, **v["biometrics"]} for k, v in all_users.items()]
    data = {"users": users_list}
    
    return data

@router.get("/{user}")
async def user_biometrics(req: Request, res: Response, user: str):
    user_info = req.app.user_cache.get(user)
    if not user_info:
        res.status_code = status.HTTP_404_NOT_FOUND
        return {"error": f"User {user} not found"}
    return user_info["biometrics"]

@router.post("/{user}/update_biometrics")
async def update_user_biometrics(req: Request, res: Response, user: str, new_biometrics: Biometrics, db: Session = Depends(get_db)):
    user_data = req.app.user_cache.update_biometrics(user, new_biometrics, db)
    if user_data is None:
        res.status_code = status.HTTP_404_NOT_FOUND
        return {"error": f"User {user} not found"}
    return { "message": f"Successfully updated biometrics for {user}"}