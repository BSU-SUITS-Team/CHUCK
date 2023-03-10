import random
import string

from fastapi import APIRouter, Request, status
from pydantic import BaseModel

router = APIRouter(prefix="/location", tags=["location"])

random.seed(20031101)


class LocationLLAH(BaseModel):
    latitude: float
    longitude: float
    altitude: float
    heading: float


def create_random_location(
    latitude_min=-90,
    latitude_max=90,
    longitude_min=-180,
    longitude_max=180,
    altitude_min=420,
    altitude_max=8848,
    heading_min=0,
    heading_max=360,
):
    latitude = round(random.uniform(latitude_min, latitude_max), 6)
    longitude = round(random.uniform(longitude_min, longitude_max), 6)
    altitude = random.randint(altitude_min, altitude_max)
    heading = random.randint(heading_min, heading_max)
    return (latitude, longitude, altitude, heading)


def create_random_string(characters=string.ascii_lowercase):
    return "".join(random.choice(characters) for _ in range(6))


@router.get("/")
async def location(request: Request):
    all_users = request.app.user_cache.get_all()
    users_list = [{**{"user": k}, **v["location"]} for k, v in all_users.items()]
    data = {"users": users_list}

    return data


@router.get("/{user}")
async def user_location(request: Request, user: str):
    user_info = request.app.user_cache.get(user)
    return user_info["location"] if user_info else status.HTTP_404_NOT_FOUND


@router.post("/{user}/update_location")
async def update_user_location(request: Request, user: str, new_location: LocationLLAH):
    request.app.user_cache.update_location(user, new_location)
