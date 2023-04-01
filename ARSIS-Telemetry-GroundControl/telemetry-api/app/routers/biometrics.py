import random

from fastapi import APIRouter, Request, status, Response
from pydantic import BaseModel
from app.database import connection

router = APIRouter(
    prefix="/biometrics",
    tags=["biometrics"]
)

random.seed(20031101)

class Biometrics(BaseModel):
    bpm: int
    o2: int
    battery: int

keys = ['id', 'bpm', 'o2', 'battery', 'createdAt', 'updatedAt']

@router.get("/")
async def get_biometrics(request: Request):
    all_users = request.app.user_cache.get_all()
    users_list = [{**{"user": k}, **v["biometrics"]} for k, v in all_users.items()]
    data = {"users": users_list}
    
    return data

@router.get("/{user_id}")
async def user_biometrics(req: Request, res: Response, user_id: int):
    with connection.cursor() as db:
        query = f"SELECT * FROM biometrics WHERE id = {user_id} ORDER BY createdat DESC LIMIT 1"
        db.execute(query)
        row = db.fetchone()
        if row is None:
            res.status_code = status.HTTP_404_NOT_FOUND
            return {"error": f"User with id: {user_id} not found"}
        return {i: j for i, j in zip(keys, row[1:])}

@router.post("/{user_id}/update_biometrics")
async def update_user_biometrics(req: Request, res: Response, user_id: int, new_biometrics: Biometrics):
    with connection.cursor() as db:
        keys_post = f"{keys[0]}, {keys[1]}, {keys[2]}, {keys[3]}"
        values = f"{user_id}, {new_biometrics.bpm}, {new_biometrics.o2}, {new_biometrics.battery}"
        query = f"INSERT INTO biometrics ({keys_post}) VALUES ({values}) RETURNING *;"
        db.execute(query)
        row = db.fetchone()
        connection.commit()
        return {i: j for i, j in zip(keys, row[1:])}