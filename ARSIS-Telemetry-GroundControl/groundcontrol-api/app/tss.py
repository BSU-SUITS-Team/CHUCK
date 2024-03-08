import requests
import logging
from os import getenv

endpoint = getenv("TSS_ENDPOINT")
get = {
    "telemetry": "/json_data/teams/0/TELEMETRY.json",
    "rover": "/json_data/ROVER.json",
    "imu": "/json_data/IMU.json",
    "uia": "/json_data/UIA.json",
    "dcu": "/json_data/DCU.json",
    "eva": "/json_data/teams/0/EVA.json",
}

tss_keys = get.keys()


async def get_from_tss(key):
    logging.info(f"MCP: GET request to {endpoint + get[key]}")
    response = requests.get(endpoint + get[key])
    return response
