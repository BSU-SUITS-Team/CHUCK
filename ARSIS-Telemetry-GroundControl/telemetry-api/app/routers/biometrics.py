from fastapi import APIRouter
import random

router = APIRouter()

bpm = 120
o2 = 100
battery = 100

o2_drop_chance = 25
battery_drop_chance = 25

def evaluate_biometrics():
    global bpm
    global battery
    global o2

    bpm = random.randrange(95, 162)
    if random.randint(0, 100) < battery_drop_chance and battery > 0:
        battery -= random.randrange(0, 2)
    elif battery <= 0:
        battery = 0
    if random.randint(0, 100) < o2_drop_chance and o2 > 0:
        o2 -= random.randrange(0, 2)
    elif o2 <= 0:
        o2 = 0

@router.get("/biometrics/", tags=["biometrics"])
async def get_biometrics():
    evaluate_biometrics()
    return {"bpm": bpm, "o2": o2, "battery": battery}

@router.get("/biometrics/{data_type}", tags=["biometrics"])
async def get_biometrics(data_type: str):
    evaluate_biometrics()
    if not data_type:
        return {"message": "No data type specified"}
    match data_type:
        case "bpm":
            return { "bpm": bpm }
        case "o2":
            return { "o2": o2 }
        case "battery":
            return { "battery": battery }
        case _:
            return {"error": "Invalid data type"}
        
        