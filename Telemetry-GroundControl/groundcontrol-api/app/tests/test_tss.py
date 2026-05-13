import responses
import json
from app.tss import endpoint, get, get_from_tss
import pytest


@pytest.mark.asyncio(scope="class")
class TestTSSRequests:
    @responses.activate
    async def test_get_telemetry(self):
        with open("app/tests/json/sample_telemetry.json") as file:
            body = json.load(file)
            responses.add(
                **{
                    "method": responses.GET,
                    "url": endpoint + get["telemetry"],
                    "json": body,
                    "status": 200,
                }
            )
            response = await get_from_tss("telemetry")
            assert response.status_code == 200
            assert response.json()["telemetry"] == body["telemetry"]

    @responses.activate
    async def test_get_imu(self):
        with open("app/tests/json/sample_telemetry.json") as file:
            body = json.load(file)
            responses.add(
                **{
                    "method": responses.GET,
                    "url": endpoint + get["imu"],
                    "json": body,
                    "status": 200,
                }
            )
            response = await get_from_tss("imu")
            assert response.status_code == 200
            assert response.json()["imu"] == body["imu"]

    @responses.activate
    async def test_get_rover(self):
        with open("app/tests/json/sample_rover.json") as file:
            body = json.load(file)
            responses.add(
                **{
                    "method": responses.GET,
                    "url": endpoint + get["rover"],
                    "json": body,
                    "status": 200,
                }
            )
            response = await get_from_tss("rover")
            assert response.status_code == 200
            assert response.json()["rover"] == body["rover"]

    @responses.activate
    async def test_get_dcu(self):
        with open("app/tests/json/sample_telemetry.json") as file:
            body = json.load(file)
            responses.add(
                **{
                    "method": responses.GET,
                    "url": endpoint + get["dcu"],
                    "json": body,
                    "status": 200,
                }
            )
            response = await get_from_tss("dcu")
            assert response.status_code == 200
            assert response.json()["dcu"] == body["dcu"]

    @responses.activate
    async def test_get_uia(self):
        with open("app/tests/json/sample_telemetry.json") as file:
            body = json.load(file)
            responses.add(
                **{
                    "method": responses.GET,
                    "url": endpoint + get["uia"],
                    "json": body,
                    "status": 200,
                }
            )
            response = await get_from_tss("uia")
            assert response.status_code == 200
            assert response.json()["uia"] == body["uia"]

    @responses.activate
    async def test_get_eva(self):
        with open("app/tests/json/sample_telemetry.json") as file:
            body = json.load(file)
            responses.add(
                **{
                    "method": responses.GET,
                    "url": endpoint + get["eva"],
                    "json": body,
                    "status": 200,
                }
            )
            response = await get_from_tss("eva")
            assert response.status_code == 200
            assert response.json()["eva"] == body["status"]
