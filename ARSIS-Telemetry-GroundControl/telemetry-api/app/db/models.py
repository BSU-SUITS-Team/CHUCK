import time
from sqlalchemy import Column, INTEGER, TIMESTAMP, String, ForeignKey
from sqlalchemy.orm import relationship

from .database import Base

# class Biometrics(Base):
#     __tablename__ = "biometrics"

#     uuid = Column(INTEGER, primary_key=True, index=True)
#     timein = Column(TIMESTAMP, index=True)
#     user_id = Column(String(20), ForeignKey("users.user_id"))
#     o2 = Column(INTEGER, index=True)
#     battery = Column(INTEGER, index=True)
#     bpm = Column(INTEGER, index=True)

#     def __init__(self, userid, o2, battery, bpm):
#         self.user_id = userid
#         self.timein = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(time.time())) # year-month-day hour:minute:second
#         self.o2 = o2
#         self.battery = battery
#         self.bpm = bpm

#     user = relationship("User", back_populates="biometrics")

    
# class Location(Base):
#     __tablename__ = "location"

#     uuid = Column(INTEGER, primary_key=True, index=True)
#     timein = Column(TIMESTAMP, index=True)
#     user_id = Column(String(20), ForeignKey("users.user_id"))
#     latitude = Column(INTEGER, index=True)
#     longitude = Column(INTEGER, index=True)
#     altitude = Column(INTEGER, index=True)
#     heading = Column(INTEGER, index=True)

#     def __init__(self, userid, latitude, longitude, altitude, heading):
#         self.user_id = userid
#         self.timein = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(time.time())) # year-month-day hour:minute:second
#         self.latitude = latitude
#         self.longitude = longitude
#         self.altitude = altitude
#         self.heading = heading

#     def to_json(self):
#         return dict(userid=self.userid, location=self.get_location())
    
#     def get_location(self):
#         return dict(latitude=self.latitude, longitude=self.longitude, altitude=self.altitude, heading=self.heading)
    
#     user = relationship("User", back_populates="location")

class User(Base):
    __tablename__ = "users"

    uuid = Column(INTEGER, primary_key=True, index=True)
    name = Column(String(20), index=True)

    def __init__(self, name):
        self.name = name
    
    # biometrics = relationship("Biometrics", back_populates="user")
    # location = relationship("Location", back_populates="user")