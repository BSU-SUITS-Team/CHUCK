import requests
import responses
import json
from app.tss import endpoint, get, get_from_tss

class TestTSSRequests:

    @responses.activate
    async def test_get_telemetry(self):
        with open("app/tests/json/sample_telemetry.json") as file:
            body = json.load(file)
            responses.add(**{
                "method": responses.GET,
                "url": endpoint + get["telemetry"],
                "body": str(body),
                "status": 200,
                "content_type": "application/json",
            })
            response = await get_from_tss("telemetry")
            assert response.status_code == 200

    @responses.activate
    async def test_get_imu(self):
        with open("app/tests/json/sample_imu.json") as file:
            body = json.load(file)
            responses.add(**{
                "method": responses.GET,
                "url": endpoint + get["imu"],
                "body": str(body),
                "status": 200,
                "content_type": "application/json",
            })
            response = await get_from_tss("imu")
            assert response.status_code == 200

    @responses.activate
    async def test_get_rover(self):
        with open("app/tests/json/sample_rover.json") as file:
            body = json.load(file)
            responses.add(**{
                "method": responses.GET,
                "url": endpoint + get["rover"],
                "body": str(body),
                "status": 200,
                "content_type": "application/json",
            })
            response = await get_from_tss("rover")
            assert response.status_code == 200

    @responses.activate
    async def test_get_dcu(self):
        with open("app/tests/json/sample_dcu.json") as file:
            body = json.load(file)
            responses.add(**{
                "method": responses.GET,
                "url": endpoint + get["dcu"],
                "body": str(body),
                "status": 200,
                "content_type": "application/json",
            })
            response = await get_from_tss("dcu")
            assert response.status_code == 200

    @responses.activate
    async def test_get_uia(self):
        with open("app/tests/json/sample_uia.json") as file:
            body = json.load(file)
            responses.add(**{
                "method": responses.GET,
                "url": endpoint + get["uia"],
                "body": str(body),
                "status": 200,
                "content_type": "application/json",
            })
            response = await get_from_tss("uia")
            assert response.status_code == 200

    @responses.activate
    async def test_get_eva(self):
        with open("app/tests/json/sample_eva.json") as file:
            body = json.load(file)
            responses.add(**{
                "method": responses.GET,
                "url": endpoint + get["eva"],
                "body": str(body),
                "status": 200,
                "content_type": "application/json",
            })
            response = await get_from_tss("eva")
            assert response.status_code == 200
