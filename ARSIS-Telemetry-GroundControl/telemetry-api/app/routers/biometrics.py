import string
import random

from fastapi import APIRouter, Request, status
from pydantic import BaseModel

router = APIRouter(
    prefix="/biometrics",
    tags=["biometrics"]
)

random.seed(20031101)

class Biometric(BaseModel):
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

@router.get("/", tags=["biometrics"])
async def get_biometrics(request: Request):
    all_users = request.app.user_cache.get_all()
    users_list = [{**{"user": k}, **v["biometrics"]} for k, v in all_users.items()]
    data = {"users": users_list}
    return data

@router.get("/{user}", tags=["biometrics"])
async def get_biometrics(request: Request, user: str):
    user_info = request.app.user_cache.get(user)
    return user_info["biometrics"] if user_info else status.HTTP_404_NOT_FOUND

@router.get("/{user}/update_biometrics", tags=["biometrics"])
async def get_biometrics(request: Request, user: str, new_biometric: Biometric):
    request.app.user_cache.update_biometrics(user, new_biometric)