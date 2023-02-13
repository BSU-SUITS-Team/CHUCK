from fastapi import FastAPI
from pydantic import BaseModel
import random

app = FastAPI()

@app.get("/")
async def root():
    return {"message": "Ground Control API"}


@app.get("/fake-procedure")
def fake_procedure():
    return {"name": "procedure 1", "number_of_tasks": 5}


@app.get("/fake-task")
def fake_task():
    to_return = []
    for i in range(random.randint(1, 7)):
        to_return.append(["text", "space " * random.randint(6, 15)])
    return {"task_items": to_return}


class LoggingRequest(BaseModel):
    data: str


@app.put("/logger/", status_code=200)
async def gs_logger(data: LoggingRequest):
    print(data)
    return data
