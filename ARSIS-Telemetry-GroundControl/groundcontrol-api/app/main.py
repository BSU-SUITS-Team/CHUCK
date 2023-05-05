from fastapi import FastAPI
from app.routers import procedures
from app.routers import logs
from app.routers import navigation

app = FastAPI()
app.include_router(procedures.router)
app.include_router(logs.router)
app.include_router(navigation.router)


@app.get("/")
async def root():
    return {"message": "Ground Control API"}
