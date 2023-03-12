import sys
from .. import crud
from ..db.database import get_db
from fastapi import Depends
from sqlalchemy.orm import Session
class UserCache:
    def __init__(self):
        self.users = {}
        self.max_size = 1000000000; # 1 gigabyte

    def create_new_user_dict(self):
        bpm = 100
        o2 = 100
        battery = 100
        latitude = 10
        longitude = 100
        altitude = 0
        heading = 0
        return {
            "biometrics": {"bpm": bpm, "o2": o2, "battery": battery},
            "location": {
                "latitude": latitude,
                "longitude": longitude,
                "altitude": altitude,
                "heading": heading,
            },
        }
    
    def dump_to_db(self, db: Session):
        print("running")
        for k, v in self.users.items():
            crud.create_user(db, k)
            crud.create_user_biometrics(db, k, v["biometrics"])
            crud.create_user_location(db, k, v["location"])
        self.users.clear()
        
    def check_size(self, db: Session):
        size = sys.getsizeof(self) # returns size in bytes
        if size >= self.max_size:
            self.dump_to_db(db)

    def get_all(self):
        return self.users

    def get(self, user_id):
        return self.users.get(user_id, None)

    def register(self, user_id, db: Session):
        if user_id in self.users:
            return None
        self.users[user_id] = self.create_new_user_dict()
        self.check_size(db)
        return user_id

    def update_location(self, user_id, new_location):
        self.users[user_id]["location"] = new_location

    def update_biometrics(self, user_id, new_biometrics):
        self.users[user_id]["biometrics"] = new_biometrics
