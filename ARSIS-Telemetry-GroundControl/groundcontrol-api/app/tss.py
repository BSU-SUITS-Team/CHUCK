import requests
import logging
from os import getenv

endpoint = getenv("TSS_ENDPOINT")
get = {
    "telemetry" : "/json_data/teams/0/TELEMETRY.json",
    "rover"     : "/json_data/ROVER.json",
    "imu"       : "/json_data/IMU.json",
    "uia"       : "/json_data/UIA.json",
    "dcu"       : "/json_data/DCU.json",
    "eva"       : "/json_data/teams/0/EVA.json",
}

def get_telemetry():
    logging.info(f"MCP: GET request to {endpoint + get['telemetry']}")
    response = requests.get(endpoint + get["telemetry"])
    return response

def get_rover():
    pass

def get_imu():
    pass

def get_uia():
    pass

def get_dcu():
    pass

def get_eva():
    pass
