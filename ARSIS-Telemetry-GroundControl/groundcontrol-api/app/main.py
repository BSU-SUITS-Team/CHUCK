from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from app.routers import procedures
from app.routers import logs
from app.routers import navigation
from app.routers import ws
from app.routers import chat

app = FastAPI()
app.include_router(procedures.router)
app.include_router(logs.router)
app.include_router(navigation.router)
app.include_router(ws.router)
app.include_router(chat.router)


origins = [
    "http://localhost:3000"
]

app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


@app.get("/")
async def root():
    return {"message": "Ground Control API"}
