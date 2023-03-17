import json
from fastapi import APIRouter
import requests
import re
import ipaddress


router = APIRouter(prefix="/devices", tags=["devices"])

devices = {}

@router.get("/get_configured")
async def return_all_configured_devices():
    return devices

@router.get("/get_configured/{name}")
async def return_configured_device(name: str):
    return name in devices.keys()

@router.post("/configure/{name}")
async def configure_device(name: str, ip_address: str):
    if ip_address is None:
        return 400
    
    try:
        ip_address = ipaddress.ip_address(ip_address)
        devices[name] = ip_address
    except ValueError:
        return 400
    return 200

@router.delete("/remove/{name}")
async def remove_device(name: str):
    if name in devices.keys():
        del devices[name]
        return 200
    return 400

@router.post("/broadcast")
async def broadcast_command(command: str):
    for device in devices:
        requests.put("http://" + str(devices[device]) + ":8282/" + command)
    return 200

@router.post("/send_command")
async def send_command(names: list[str], command: str):
    for name in names:
        if name in devices.keys():
            requests.put("http://" + str(devices[name]) + ":8282/" + command)
    return 200

@router.post("/send_command/{name}")
async def send_command(name: str, command: str):
    if name in devices.keys():
        requests.put(devices[name] + "/" + command)
        return 200
    return 404
