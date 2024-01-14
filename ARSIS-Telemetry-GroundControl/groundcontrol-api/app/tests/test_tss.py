import requests
import responses
import json
from app.tss import endpoint, get, get_dcu, get_telemetry, get_imu, get_rover

class TestTSSRequests:

    @responses.activate
    def test_get_telemetry(self):
        with open("app/tests/json/sample_telemetry.json") as file:
            body = json.load(file)
            responses.add(**{
                "method": responses.GET,
                "url": endpoint + get["telemetry"],
                "body": str(body),
                "status": 200,
                "content_type": "application/json",
            })
            response = get_telemetry()
            assert response.status_code == 200
    
    @responses.activate
    def test_get_imu(self):
        with open("app/tests/json/sample_imu.json") as file:
            body = json.load(file)
            responses.add(**{
                "method": responses.GET,
                "url": endpoint + get["imu"],
                "body": str(body),
                "status": 200,
                "content_type": "application/json",
            })
            response = get_imu()
            assert response.status_code == 200

    @responses.activate
    def test_get_rover(self):
        with open("app/tests/json/sample_rover.json") as file:
            body = json.load(file)
            responses.add(**{
                "method": responses.GET,
                "url": endpoint + get["rover"],
                "body": str(body),
                "status": 200,
                "content_type": "application/json",
            })
            response = get_rover()
            assert response.status_code == 200

    @responses.activate
    def test_get_dcu(self):
        with open("app/tests/json/sample_dcu.json") as file:
            body = json.load(file)
            responses.add(**{
                "method": responses.GET,
                "url": endpoint + get["dcu"],
                "body": str(body),
                "status": 200,
                "content_type": "application/json",
            })
            response = get_dcu()
            assert response.status_code == 200