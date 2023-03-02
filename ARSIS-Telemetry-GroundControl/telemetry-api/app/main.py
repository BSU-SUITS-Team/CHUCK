from fastapi import FastAPI
from app.routers import location

app = FastAPI()
app.include_router(location.router)

@app.get("/")
async def root():
    return {"message": "Telemetry API"}

# TODO: Move this into a biometrics module
@app.get("/biometrics/")
async def biometrics():
    return {"hr": 75, "o2": 0.9}
