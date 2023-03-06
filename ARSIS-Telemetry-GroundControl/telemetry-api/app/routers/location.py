import random
import string

from fastapi import APIRouter, Request
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
async def location():

    data = {"users": []}

    for _ in range(random.randint(1, 20)):
        (latitude, longitude, altitude, heading) = create_random_location()
        data["users"].append(
            {
                "user": create_random_string(),
                "latitude": latitude,
                "longitude": longitude,
                "altitude": altitude,
                "heading": heading,
            }
        )

    return data


@router.get("/{user}")
async def user_location(user: str):
    (latitude, longitude, altitude, heading) = create_random_location()

    return {
        "latitude": latitude,
        "longitude": longitude,
        "altitude": altitude,
        "heading": heading,
    }


@router.post("/{user}/location")
async def update_user_location(request: Request, user: str, new_location: LocationLLAH):
    request.app.user_cache.update_location(user, new_location)
