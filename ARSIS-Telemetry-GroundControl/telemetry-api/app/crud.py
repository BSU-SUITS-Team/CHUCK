from sqlalchemy.orm import Session

from .db import models, schemas

# def get_user(db: Session, uuid: int):
#     return db.query(models.Log).filter(models.Log.uuid == uuid).first()

# def get_users(db: Session, skip: int = 0):
#     return db.query(models.Log).offset(skip).all()

def create_user(db: Session, user: schemas.UserCreate):
    db_user = models.User(user)
    db.add(db_user)
    db.commit()
    db.refresh(db_user)
    return db_user

def create_user_biometrics(db: Session, user: schemas.UserCreate, biometrics: schemas.BiometricsCreate):
    db_biometrics = models.Biometrics(user, **biometrics)
    db.add(db_biometrics)
    db.commit()
    db.refresh(db_biometrics)
    return db_biometrics

def create_user_location(db: Session, user: schemas.UserCreate, location: schemas.LocationCreate):
    db_location = models.Location(user, **location)
    db.add(db_location)
    db.commit()
    db.refresh(db_location)
    return db_location