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

@app.put("/logger/", response_model=schemas.Log, status_code=201)
def create_log(log: schemas.LogCreate, db: Session = Depends(get_db)):
    print(log)
    return crud.create_log(db=db, log=log)

@app.get("/logger/{uuid}", status_code=200)
def get_log(uuid: int, db: Session = Depends(get_db)):
    return crud.get_log(db, uuid)
