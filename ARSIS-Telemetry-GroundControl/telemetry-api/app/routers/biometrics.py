import random

from fastapi import APIRouter, status, Response
from pydantic import BaseModel
from app.database import connection

router = APIRouter(
    prefix="/biometrics",
    tags=["biometrics"]
)

random.seed(20031101)

class Biometrics(BaseModel):
    heartrate: int
    o2: int
    battery: int
    fan: int
    vent: bool
    co2: int
    sop: bool
    suitPressure: int

keys = ['id', 'heartrate', 'o2', 'battery', 'fan', 'vent', 'co2', 'sop', 'suitPressure', 'createdAt', 'updatedAt']

@router.get("/")
async def get_biometrics():
    with connection.cursor() as db:
        db.execute("SELECT * FROM biometrics;")
        result = db.fetchall()
        response = []
        for row in result:
            response.append({i: j for i, j in zip(keys, row[1:])})
        return {"users": response}

@router.get("/{user_id}")
async def user_biometrics(res: Response, user_id: int):
    with connection.cursor() as db:
        query = f"SELECT * FROM biometrics WHERE id = {user_id} ORDER BY createdat DESC LIMIT 1"
        db.execute(query)
        row = db.fetchone()
        if row is None:
            res.status_code = status.HTTP_404_NOT_FOUND
            return {"error": f"Biometrics data for user with id: {user_id} not found"}
        return {i: j for i, j in zip(keys, row[1:])}

@router.post("/{user_id}/update_biometrics")
async def update_user_biometrics(user_id: int, new_biometrics: Biometrics):
    with connection.cursor() as db:
        keys_post = f"{keys[0]}, {keys[1]}, {keys[2]}, {keys[3]}"
        values = f"{user_id}, {new_biometrics.heartrate}, {new_biometrics.o2}, {new_biometrics.battery}, {new_biometrics.fan}, {new_biometrics.vent}, {new_biometrics.co2}, {new_biometrics.sop}, {new_biometrics.suitPressure}"
        query = f"INSERT INTO biometrics ({keys_post}) VALUES ({values}) RETURNING *;"
        db.execute(query)
        row = db.fetchone()
        connection.commit()
        return {i: j for i, j in zip(keys, row[1:])}