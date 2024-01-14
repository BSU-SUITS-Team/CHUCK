import requests
import responses
import json
from app.tss import endpoint, get, get_telemetry

class TestTSSRequests:

    @responses.activate
    def test_get_telemetry(self):
        with open("app/tests/sample_telemetry.json") as file:
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