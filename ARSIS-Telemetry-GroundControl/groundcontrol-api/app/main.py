from fastapi import FastAPI
from app.routers import procedures, logs, chat

app = FastAPI()
app.include_router(procedures.router)
app.include_router(logs.router)
app.include_router(chat.router)

@app.get("/")
async def root():
    return {"message": "Ground Control API"}




