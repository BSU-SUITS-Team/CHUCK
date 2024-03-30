import asyncio

from pydantic import BaseModel
from app.datastore import ds
from fastapi import APIRouter
# from app.routers.on_server_navigation.default_paths import paths, paths_json
from app.routers.on_server_navigation.create_path import CreatePoint
import rasterio
import uuid
from app.event import Event

router = APIRouter(prefix="/navigation", tags=["navigation"])
dataset = rasterio.open("/code/app/routers/rockyard_map_geo.tif")
pins_key = "pins"

class Point(BaseModel):
    name: str | None
    x: int | None
    y: int | None
    lat: float | None
    lon: float | None

@router.get("/" + pins_key)
async def procedures():
    return {pins_key: ds.cache.get(pins_key, [])}

async def add_pin(point: CreatePoint, name: str):
    pin_event = Event.create_event(pins_key, point.get_dict(), upsert_key=name)
    await ds.add_event(pins_key, pin_event)
    print(ds.cache.get(pins_key, None))

@router.post("/")
def translate(point: dict):
    name = point.get("name", str(uuid.uuid4()))
    x, y = (point.get("x", None), point.get("y", None))
    lat, lon = (point.get("lat", None), point.get("lon", None))
    # do not use "falsy" values (e.g. 0 is evaluated as false)
    if x != None and y != None:
        lon, lat = dataset.xy(x, y) # convert pixel to lat/lon
    elif lat != None and lon != None:
        y, x = ~dataset.transform * (lon, lat) # convert lat/lon to pixel
    else:
        print("Not able to parse pin coordinates!")
        return {"message": "Failed to create pin. Not able to parse pin coordinates!"}
    pin = CreatePoint(x=x, y=y, lat=lat, lon=lon, altitude=None)
    asyncio.run(add_pin(pin, name))
    return {"message": "Successfully created pin."}