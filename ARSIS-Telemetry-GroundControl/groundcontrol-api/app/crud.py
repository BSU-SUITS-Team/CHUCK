from sqlalchemy.orm import Session

from .db import models, schemas

def get_log(db: Session, uuid: int):
    return db.query(models.Log).filter(models.Log.uuid == uuid).first()

def get_log_by_time(db: Session, time: str):
    return db.query(models.Log).filter(models.Log.time == time).first()

def get_logs(db: Session, skip: int = 0, limit: int = 10):
    return db.query(models.Log).offset(skip).limit(limit).all()

def create_log(db: Session, log: schemas.LogCreate):
    db_log = models.Log(**log.dict())
    db.add(db_log)
    db.commit()
    db.refresh(db_log)
    return db_log