from fastapi import APIRouter
import random

router = APIRouter()

bpm = random.random(95, 162)
o2 = 100
battery = 100

o2_drop_chance = 50
battery_drop_chance = 50

@router.get("/biometrics/", tag=["biometrics"])
async def get_biometrics():
    if random.randint(0, 100) < battery_drop_chance:
        battery -= random.random(0, 2)
    if random.randint(0, 100) < o2_drop_chance:
        o2 -= random.random(0, 2)
    return {"bpm": bpm, "o2": o2, "battery": battery}

@router.get("/biometrics/{data_type}", tag=["biometrics"])
async def get_biometrics(data_type: str):
    return { "{data_type}": data_type }