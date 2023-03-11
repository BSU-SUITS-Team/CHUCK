from fastapi import FastAPI
import MicrocontrollerInterface

app = FastAPI()
rover = MicrocontrollerInterface.MicrocontrollerInterface()


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
