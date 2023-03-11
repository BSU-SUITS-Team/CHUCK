from sqlalchemy import JSON, Column, INTEGER, TIMESTAMP, String

from .database import Base

admin = [
    "suits_ui",
    "suits_telem",
    "suits_event"
]


class BiometricLog(Base):
    __tablename__ = "biometrics"

    uuid = Column(INTEGER, primary_key=True, index=True)
    timein = Column(TIMESTAMP, index=True)
    userid = Column(INTEGER, index=True)
    o2 = Column(INTEGER, index=True)
    battery = Column(INTEGER, index=True)
    bpm = Column(INTEGER, index=True)

    def __init__(self, userid, latitude, longitude, altitude, heading):
        self.userid = userid
        self.latitude = latitude
        self.longitude = longitude
        self.altitude = altitude
        self.heading = heading

    
class LocationLog(Base):
    __tablename__ = "location"

    uuid = Column(INTEGER, primary_key=True, index=True)
    timein = Column(TIMESTAMP, index=True)
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

class User(Base):
    __tablename__ = "users"

    uuid = Column('user_id', INTEGER, primary_key=True, index=True)
    username = Column(String(20), index=True)

    def __init__(self, username):
        self.username = username
        self.is_admin = username in admin

    def to_json(self):
        return dict(username=self.username, is_admin=self.is_admin)
    
    @property
    def is_admin(self):
        return self.username in admin