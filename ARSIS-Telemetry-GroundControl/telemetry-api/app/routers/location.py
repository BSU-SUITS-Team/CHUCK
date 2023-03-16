import random

from fastapi import APIRouter, Request, status, Response
from pydantic import BaseModel

router = APIRouter(prefix="/location", tags=["location"])

random.seed(20031101)


class LocationLLAH(BaseModel):
    latitude: float
    longitude: float
    altitude: float
    heading: float


# def create_random_location(
#     latitude_min=-90,
#     latitude_max=90,
#     longitude_min=-180,
#     longitude_max=180,
#     altitude_min=420,
#     altitude_max=8848,
#     heading_min=0,
#     heading_max=360,
# ):
#     latitude = round(random.uniform(latitude_min, latitude_max), 6)
#     longitude = round(random.uniform(longitude_min, longitude_max), 6)
#     altitude = random.randint(altitude_min, altitude_max)
#     heading = random.randint(heading_min, heading_max)
#     return (latitude, longitude, altitude, heading)


# def create_random_string(characters=string.ascii_lowercase):
#     return "".join(random.choice(characters) for _ in range(6))


@router.get("/")
async def location(request: Request):
    all_users = request.app.user_cache.get_all()
    users_list = [{**{"user": k}, **v["location"]} for k, v in all_users.items()]
    data = {"users": users_list}

    return data


@router.get("/{user}")
async def user_location(req: Request, res: Response, user: str):
    user_info = req.app.user_cache.get(user)
    if not user_info:
        res.status_code = status.HTTP_404_NOT_FOUND
        return {"error": f"User {user} not found"}
    return user_info["location"]


@router.post("/{user}/update_location")
async def update_user_location(req: Request, res: Response, user: str, new_location: LocationLLAH):
    user_data = req.app.user_cache.update_location(user, new_location)
    if user_data is None:
        res.status_code = status.HTTP_404_NOT_FOUND
        return {"error": f"User {user} not found"}
    return { "message": f"Successfully updated location for {user}"}