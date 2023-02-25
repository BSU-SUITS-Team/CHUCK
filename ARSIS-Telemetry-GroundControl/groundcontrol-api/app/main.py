from fastapi import Depends, FastAPI, HTTPException
from sqlalchemy.orm import Session
from pydantic import BaseModel

from .db.database import SessionLocal, engine
from .db import models, schemas
from . import crud

models.Base.metadata.create_all(bind=engine)

app = FastAPI()

# Dependency
def get_db():
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()


@app.get("/")
async def root():
    return {"message": "Ground Control API"}

@app.post("/logger/", response_model=schemas.Log, status_code=200)
def create_user(log: schemas.LogCreate, db: Session = Depends(get_db)):
    print(log)
    return crud.create_log(db=db, log=log)

# class LoggingRequest(BaseModel):
#     data: str

# @app.put("/logger/", status_code=200)
# async def gs_logger(data: LoggingRequest):
#     print(data)
#     return data
