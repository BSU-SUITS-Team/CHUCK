import requests
import logging
from os import getenv

endpoint = getenv("TSS_ENDPOINT", "http://localhost:14141")
get = {
    "telemetry": "/data/EVA.json",
    "rover": "/data/ROVER.json",
    "imu": "/data/EVA.json",
    "uia": "/data/EVA.json",
    "dcu": "/data/EVA.json",
    "eva": "/data/EVA.json",
}

tss_keys = get.keys()

bundled_eva_keys = {
    "telemetry": "telemetry",
    "imu": "imu",
    "uia": "uia",
    "dcu": "dcu",
    "eva": "status",
}


class ParsedTSSResponse:
    def __init__(self, response, payload):
        self.response = response
        self.status_code = response.status_code

        self._payload = payload

    def json(self):
        return self._payload

    def __getattr__(self, name):
        return getattr(self.response, name)


def parse_tss_payload(key, payload):
    if key in payload:
        return {key: payload[key]}

    source_key = bundled_eva_keys.get(key)
    if source_key and source_key in payload:
        return {key: payload[source_key]}

    return {key: payload}


async def get_from_tss(key):
    logging.info(f"MCP: GET request to {endpoint + get[key]}")
    response = requests.get(endpoint + get[key])
    if response.status_code != 200:
        return response

    return ParsedTSSResponse(response, parse_tss_payload(key, response.json()))
