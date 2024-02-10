import asyncio

from app.datastore import ds
from app.routers import chat, logs, navigation, procedures, ws
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

app = FastAPI()
app.include_router(procedures.router)
app.include_router(logs.router)
app.include_router(navigation.router)
app.include_router(ws.router)
app.include_router(chat.router)

origins = ["http://localhost:3000"]

app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


@app.on_event("startup")
async def startup_event():
    asyncio.create_task(ds.start_polling())


@app.get("/")
async def root():
    return {"message": "Ground Control API"}
