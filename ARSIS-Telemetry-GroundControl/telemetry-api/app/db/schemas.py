from pydantic import BaseModel

class BiometricsBase(BaseModel):
    o2: int
    battery: int
    bpm: int

class Biometrics(BiometricsBase):
    uuid: int

class LocationBase(BaseModel):
    latitude: int
    longitude: int
    altitude: int
    heading: int
    
class Location(LocationBase):
    uuid: int

class UserBase(BaseModel):
    name: str

class User(UserBase):
    uuid: int

    class Config:
        orm_mode = True