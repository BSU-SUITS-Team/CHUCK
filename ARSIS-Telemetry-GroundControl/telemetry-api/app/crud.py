from sqlalchemy.orm import Session
from .db import models, schemas

def get_user(db: Session, user_id: int):
    user = db.query(models.User).filter(models.User.uuid == user_id).first()
    if user is None:
        return None
    loc = user.location.pop(0) if user.location else None
    bio = user.biometrics.pop(0) if user.biometrics else None
    location = { "latitude": loc.latitude, "longitude": loc.longitude, "altitude": loc.altitude, "heading": loc.heading}
    biometrics = { "o2": bio.o2, "battery": bio.battery, "bpm": bio.bpm }
    to_return = { "name": user.name, "biometrics": biometrics, "location": location }
    return to_return

def get_users(db: Session, skip: int = 0):
    users = db.query(models.User).offset(skip).all()
    to_return = [] # may be a built-in way to do this with sqlalchemy but couldn't find one in docs
    for user in users:
        loc = user.location.pop(0) if user.location else None
        bio = user.biometrics.pop(0) if user.biometrics else None
        location = { "latitude": loc.latitude, "longitude": loc.longitude, "altitude": loc.altitude, "heading": loc.heading}
        biometrics = { "o2": bio.o2, "battery": bio.battery, "bpm": bio.bpm }
        to_return.append({ "name": user.name, "biometrics": biometrics, "location": location }) 
    return to_return

def register_user(db: Session, user: schemas.UserBase):
    try:
        db_user = models.User(user)
        db.add(db_user)
        db.commit()
        db.refresh(db_user)
        return db_user
    except Exception as e:
        print("Error creating user: ", e)
        return None

def update_user_biometrics(db: Session, user_id: int, biometrics: schemas.BiometricsBase):
    db_biometrics = models.Biometrics(user_id, **biometrics)
    db.add(db_biometrics)
    db.commit()
    db.refresh(db_biometrics)
    return db_biometrics

def update_user_location(db: Session, user_id: int, location: schemas.LocationBase):
    db_location = models.Location(user_id, **location)
    db.add(db_location)
    db.commit()
    db.refresh(db_location)
    return db_location