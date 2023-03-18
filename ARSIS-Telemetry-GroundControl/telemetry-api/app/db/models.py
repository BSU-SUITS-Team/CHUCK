import time
from sqlalchemy import Column, INTEGER, TIMESTAMP, String, ForeignKey
from sqlalchemy.orm import relationship

from .database import Base

class Biometrics(Base):
    __tablename__ = "biometrics"

    uuid = Column(INTEGER, primary_key=True, index=True)
    timein = Column(TIMESTAMP, index=True)
    user_id = Column(INTEGER, ForeignKey("users.uuid"))
    o2 = Column(INTEGER, index=True)
    battery = Column(INTEGER, index=True)
    bpm = Column(INTEGER, index=True)

    def __init__(self, user_id, o2, battery, bpm):
        self.user_id = user_id
        self.timein = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(time.time())) # year-month-day hour:minute:second
        self.o2 = o2
        self.battery = battery
        self.bpm = bpm

    user = relationship("User", back_populates="biometrics")

class Location(Base):
    __tablename__ = "location"

    uuid = Column(INTEGER, primary_key=True, index=True)
    timein = Column(TIMESTAMP, index=True)
    user_id = Column(INTEGER, ForeignKey("users.uuid"))
    latitude = Column(INTEGER, index=True)
    longitude = Column(INTEGER, index=True)
    altitude = Column(INTEGER, index=True)
    heading = Column(INTEGER, index=True)

    def __init__(self, user_id, latitude, longitude, altitude, heading):
        self.user_id = user_id
        self.timein = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(time.time())) # year-month-day hour:minute:second
        self.latitude = latitude
        self.longitude = longitude
        self.altitude = altitude
        self.heading = heading
    
    user = relationship("User", back_populates="location")

class User(Base):
    __tablename__ = "users"

    uuid = Column(INTEGER, primary_key=True, index=True)
    name = Column(String(20), index=True)

    def __init__(self, name):
        self.name = name
    
    biometrics = relationship("Biometrics", back_populates="user")
    location = relationship("Location", back_populates="user")