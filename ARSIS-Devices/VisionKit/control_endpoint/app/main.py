from fastapi import FastAPI
import MicrocontrollerInterface
import requests
import socket

NAME="rover"

app = FastAPI()
rover = MicrocontrollerInterface.MicrocontrollerInterface()
ip = socket.gethostbyname(socket.gethostname())
GATEWAY_IP = "192.168.137.1"
requests.post(f"http://{GATEWAY_IP}:8181/devices/configure/{NAME}?ip_address={ip}")

@app.get("/")
async def root():
    return {"message": "Rover API"}

@app.put("/{motors}")
async def move(motors: str):
    # motors will be in the format byte:byte
    # where byte is a integer between 0 and 255

    # validate
    try:
        right, left = (int(val) for val in motors.split(":"))
        if right < 0 or right > 255 or left < 0 or left > 255:
            raise ValueError
        rover.send(bytearray([left, right]))
    except ValueError:
        return {"message": "Invalid motor values"}
    except Exception:
        return {"message": "Error sending command to rover"}
