from fastapi import FastAPI
from .routers import biometrics

app = FastAPI()

app.include_router(biometrics.router)

@app.get("/")
async def root():
    return {"message": "Telemetry API"}


# TODO: Move this into a locations module
@app.get("/location/")
async def location():
    return {"lat": 10, "lon": 100}
