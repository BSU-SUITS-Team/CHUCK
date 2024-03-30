import asyncio

from pydantic import BaseModel
from app.datastore import ds
from fastapi import APIRouter
# from app.routers.on_server_navigation.default_paths import paths, paths_json
from app.routers.on_server_navigation.create_path import CreatePath, CreatePoint
import rasterio
import uuid
from app.event import Event

router = APIRouter(prefix="/navigation", tags=["navigation"])
dataset = rasterio.open("/code/app/routers/rockyard_map_geo.tif")
pins_key = "pins"
paths_key = "paths"

@router.get("/" + pins_key)
async def pins():
    return {pins_key: ds.cache.get(pins_key, [])}

@router.get("/" + paths_key)
async def paths():
    return {paths_key: ds.cache.get(paths_key, [])}

class Point(BaseModel):
    name: str = str(uuid.uuid4())
    x: int | None = None
    y: int | None = None
    lat: float | None = None
    lon: float | None = None

async def add_pin(point: CreatePoint, name: str):
    pin_event = Event.create_event(pins_key, point.get_dict(), upsert_key=name)
    await ds.add_event(pins_key, pin_event)

@router.post("/" + pins_key)
def translate(point: Point):
    x, y = (point.x, point.y)
    lat, lon = (point.lat, point.lon)
    # do not use "falsy" values (e.g. 0 is evaluated as false)
    if x != None and y != None:
        lon, lat = dataset.xy(x, y) # convert pixel to lat/lon
    elif lat != None and lon != None:
        y, x = ~dataset.transform * (lon, lat) # convert lat/lon to pixel
    else:
        print("Not able to parse pin coordinates!")
        return {"message": "Failed to create pin. Not able to parse pin coordinates!"}
    pin = CreatePoint(x=x, y=y, lat=lat, lon=lon, altitude=None)
    asyncio.run(add_pin(pin, point.name))
    return {"message": "Successfully created pin."}

class Path(BaseModel):
    name: str = str(uuid.uuid4())
    pins: list[Point]

async def add_path(path: CreatePath, name: str):
    path_event = Event.create_event(paths_key, path.get_dict(), upsert_key=name)
    await ds.add_event(paths_key, path_event)

@router.post("/" + paths_key)
def create_path(path: Path):
    new_path = CreatePath(path.name)
    for pin in path.pins:
        new_path.add_point(pin)
    asyncio.run(add_path(new_path, path.name))
    return {"message": "Successfully created path."}