from fastapi import FastAPI, Form
from pydantic import BaseModel

app = FastAPI()


@app.get("/")
async def root():
    return {"message": "Ground Control API"}


class LoggingRequest(BaseModel):
    data: str


@app.put("/logger/", status_code=200)
async def gs_logger(data: LoggingRequest):
    print(data)
    return data
