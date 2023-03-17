from fastapi import FastAPI
import MicrocontrollerInterface
import requests
import socket

NAME="rover"

app = FastAPI()
rover = MicrocontrollerInterface.MicrocontrollerInterface()
ip = input("My Ip Addr: ")
GATEWAY_IP = input("Enter groundcontroll ip: ")
print("connecting to: ", GATEWAY_IP)
requests.post(f"http://{GATEWAY_IP}:8181/devices/configure/{NAME}?ip_address={ip}")


@app.get("/")
async def root():
    return {"message": "Rover API"}

@app.put("/{motors}")
async def move(motors: str):
    # motors will be in the format byte:byte
    # where byte is a number between 0 and 255

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
