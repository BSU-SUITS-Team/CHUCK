from fastapi import FastAPI
from app.routers import location
from app.routers import biometrics

app = FastAPI()
app.include_router(location.router)
app.include_router(biometrics.router)

@app.get("/")
async def root():
    return {"message": "Telemetry API"}
