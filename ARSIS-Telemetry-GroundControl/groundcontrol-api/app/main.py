from fastapi import FastAPI, Form
from pydantic import BaseModel

app = FastAPI()


@app.get("/")
async def root():
    return {"message": "Ground Control API"}

class Logging(BaseModel):
    name: str


@app.post("/logger/", status_code=200)
async def gs_logger(data: str = Form()):
    print(data)
