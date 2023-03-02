from fastapi import APIRouter
import string
import random

router = APIRouter(
    prefix="/biometrics",
    tags=["biometrics"]
)

random.seed(20031101)

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

def create_random_string(characters = string.ascii_lowercase):
    return ''.join(random.choice(characters) for _ in range(6))

@router.get("/", tags=["biometrics"])
async def get_biometrics():
    data = {
        "users": []
    }

    for _ in range(random.randint(1, 20)):
        evaluate_biometrics()
        data["users"].append({
            "user": create_random_string(),
            "bpm": bpm, 
            "o2": o2, 
            "battery": battery
        })

    return data

@router.get("/{user}", tags=["biometrics"])
async def get_biometrics(user: str):
    evaluate_biometrics()
    return {"bpm": bpm, "o2": o2, "battery": battery}

@router.get("/{user}/{data_type}", tags=["biometrics"])
async def get_biometrics(user: str, data_type: str):
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