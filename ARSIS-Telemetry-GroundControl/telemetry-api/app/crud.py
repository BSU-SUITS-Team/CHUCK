from sqlalchemy.orm import Session

from .db import models, schemas

# def get_user(db: Session, uuid: int):
#     return db.query(models.Log).filter(models.Log.uuid == uuid).first()

# def get_users(db: Session, skip: int = 0):
#     return db.query(models.Log).offset(skip).all()

def create_user(db: Session, user: schemas.UserCreate):
    print("Creating user: " + user)
    db_user = models.User(user)
    db.add(db_user)
    db.commit()
    db.refresh(db_user)
    return db_user

def create_user_biometrics(db: Session, biometric: schemas.BiometricCreate):
    db_biometric = models.Biometric(**biometric.dict())
    db.add(db_biometric)
    db.commit()
    db.refresh(db_biometric)
    return db_biometric

# def create_user_location(db: Session, user: schemas.LocationCreate):
#     db_location = models.Location(**user.dict())
#     db.add(db_location)
#     db.commit()
#     db.refresh(db_location)
#     return db_location