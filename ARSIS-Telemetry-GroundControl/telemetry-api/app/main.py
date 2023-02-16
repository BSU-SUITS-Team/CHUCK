from fastapi import FastAPI

app = FastAPI()


@app.get("/")
async def root():
    return {"message": "Telemetry API"}


# TODO: Move this into a locations module
@app.get("/location/")
async def location():
    return {"lat": 10, "lon": 100}


# TODO: Move this into a biometrics module
@app.get("/biometrics/")
async def biometrics():
    return {"hr": 75, "o2": 0.9}
