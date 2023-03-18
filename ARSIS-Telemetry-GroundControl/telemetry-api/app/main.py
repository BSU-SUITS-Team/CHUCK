from fastapi import FastAPI
from app.routers import biometrics, location, user
from app.caches.user_cache import UserCache

from .db.database import engine
from .db import models

models.Base.metadata.create_all(bind=engine)

app = FastAPI()
app.include_router(location.router)
app.include_router(biometrics.router)
app.include_router(user.router)
app.user_cache = UserCache()


@app.get("/")
async def root():
    return {"message": "Telemetry API"}
