from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from app.routers import biometrics, location, user, uia
from fastapi.middleware.cors import CORSMiddleware
from app.caches.user_cache import UserCache

app = FastAPI()
app.include_router(location.router)
app.include_router(biometrics.router)
app.include_router(user.router)
app.include_router(uia.router)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

@app.get("/")
async def root():
    return {"message": "Telemetry API"}