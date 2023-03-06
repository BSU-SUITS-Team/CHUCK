from sqlalchemy import JSON, Column, INTEGER, TIMESTAMP, String

from .database import Base

admin = [
    "Maverick",
    "Goose"
]


class BiometricLog(Base):
    __tablename__ = "biometrics"

    uuid = Column(INTEGER, primary_key=True, index=True)
    timein = Column(TIMESTAMP, index=True)
    userid = Column(INTEGER, index=True)
    o2 = Column(INTEGER, index=True)
    battery = Column(INTEGER, index=True)
    bpm = Column(INTEGER, index=True)

class User(Base):
    __tablename__ = "users"

    uuid = Column('user_id', INTEGER, primary_key=True, index=True)
    callsign = Column(String(20), index=True)
    firstname = Column(String(20), index=True)
    lastname = Column(String(20), index=True)

    def __init__(self, callsign, firstname, lastname):
        self.callsign = callsign
        self.firstname = firstname
        self.lastname = lastname

    def to_json(self):
        return dict(callsign=self.callsign, is_admin=self.is_admin)
    
    @property
    def is_admin(self):
        return self.callsign in admin
    
class LocationLog(Base):
    __tablename__ = "location"

    uuid = Column(INTEGER, primary_key=True, index=True)
    userid = Column(String(20), index=True)
    latitude = Column(INTEGER, index=True)
    longitude = Column(INTEGER, index=True)
    altitude = Column(INTEGER, index=True)
    heading = Column(INTEGER, index=True)

    def __init__(self, userid, latitude, longitude, altitude, heading):
        self.userid = userid
        self.latitude = latitude
        self.longitude = longitude
        self.altitude = altitude
        self.heading = heading

    def to_json(self):
        return dict(userid=self.userid, location=self.get_location())
    
    def get_location(self):
        return dict(latitude=self.latitude, longitude=self.longitude, altitude=self.altitude, heading=self.heading)