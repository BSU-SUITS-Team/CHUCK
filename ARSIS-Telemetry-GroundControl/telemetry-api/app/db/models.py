import datetime
from sqlalchemy import JSON, Column, INTEGER, TIMESTAMP, String

from .database import Base

admin = [
    "suits_ui",
    "suits_telem",
    "suits_event"
]


class Biometric(Base):
    __tablename__ = "biometrics"

    uuid = Column(INTEGER, primary_key=True, index=True)
    timein = Column(TIMESTAMP, index=True)
    user_id = Column(String(20), index=True)
    o2 = Column(INTEGER, index=True)
    battery = Column(INTEGER, index=True)
    bpm = Column(INTEGER, index=True)

    def __init__(self, userid, o2, battery, bpm):
        self.user_id = userid
        self.o2 = o2
        self.battery = battery
        self.bpm = bpm

    
class Location(Base):
    __tablename__ = "location"

    uuid = Column(INTEGER, primary_key=True, index=True)
    timein = Column(TIMESTAMP, index=True)
    user_id = Column(String(20), index=True)
    latitude = Column(INTEGER, index=True)
    longitude = Column(INTEGER, index=True)
    altitude = Column(INTEGER, index=True)
    heading = Column(INTEGER, index=True)

    def __init__(self, userid, latitude, longitude, altitude, heading):
        self.user_id = userid
        self.latitude = latitude
        self.longitude = longitude
        self.altitude = altitude
        self.heading = heading

    def to_json(self):
        return dict(userid=self.userid, location=self.get_location())
    
    def get_location(self):
        return dict(latitude=self.latitude, longitude=self.longitude, altitude=self.altitude, heading=self.heading)

class User(Base):
    __tablename__ = "users"

    uuid = Column(INTEGER, primary_key=True, index=True)
    user_id = Column(String(20), index=True)

    def __init__(self, user_id):
        self.user_id = user_id

    def to_json(self):
        return dict(user_id=self.user_id)