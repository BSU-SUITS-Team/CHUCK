import random

from fastapi import APIRouter, Request, status, Response
from pydantic import BaseModel
from app.database import connection
from app.routers.user import get_user
import logging

router = APIRouter(prefix="/location", tags=["location"])

random.seed(20031101)


class LocationLLAH(BaseModel):
    latitude: float
    longitude: float
    altitude: float
    heading: float


keys = [
    "id",
    "longitude",
    "latitude",
    "altitude",
    "heading",
    "createdAt",
    "updatedAt",
]


@router.get("/")
async def location(request: Request):
    with connection.cursor() as db:
        db.execute("SELECT * FROM locations;")
        result = db.fetchall()
        response = []
        for row in result:
            response.append({i: j for i, j in zip(keys, row[1:])})
        return {"users": response}


@router.get("/{user_id}")
async def user_location(req: Request, res: Response, user_id: int):
    with connection.cursor() as db:
        query = f"SELECT * FROM locations WHERE id = {user_id} ORDER BY createdat DESC LIMIT 1;"
        db.execute(query)
        row = db.fetchone()
        return {i: j for i, j in zip(keys, row[1:])}
    user_info = req.app.user_cache.get(user_id)
    if not user_info:
        res.status_code = status.HTTP_404_NOT_FOUND
        return {"error": f"User with id: {user_id} not found"}
    return user_info["location"]


@router.post("/{user_id}/update_location")
async def update_user_location(
    req: Request, res: Response, user_id: int, new_location: LocationLLAH
):
    with connection.cursor() as db:
        keys_post = f"{keys[0]}, {keys[1]}, {keys[2]}, {keys[3]}, {keys[4]}"
        values = f"{user_id}, {new_location.longitude}, {new_location.latitude}, {new_location.altitude}, {new_location.heading}"
        query = f"INSERT INTO locations ({keys_post}) VALUES ({values}) RETURNING *;"
        logging.warn(keys_post)
        logging.warn(values)
        db.execute(query)
        row = db.fetchone()
        connection.commit()
        return {i: j for i, j in zip(keys, row[1:])}
