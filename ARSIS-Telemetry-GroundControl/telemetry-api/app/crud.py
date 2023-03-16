from sqlalchemy.orm import Session
from sqlalchemy import exc
from .db import models, schemas

def get_user(db: Session, user_id: str):
    user = db.query(models.User).filter(models.User.user_id == user_id).first()
    if user is None:
        return None
    loc = dict(user.location.__dict__)
    loc.pop('_sa_instance_state', None)
    bio = dict(user.biometrics.__dict__)
    bio.pop('_sa_instance_state', None)
    to_return = { "biometrics": bio, "location": loc }
    return to_return

# def get_users(db: Session, skip: int = 0):
#     return db.query(models.Log).offset(skip).all()

def create_user(db: Session, user: schemas.UserCreate):
    try:
        db_user = models.User(user)
        db.add(db_user)
        db.commit()
        db.refresh(db_user)
        return db_user
    except exc.IntegrityError:
        print("User already exists, skipping creation...")
        db.rollback()
        return None

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