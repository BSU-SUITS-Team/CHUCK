from pydantic import BaseModel

class BiometricsBase(BaseModel):
    user_id: str
    o2: int
    battery: int
    bpm: int

class BiometricsCreate(BiometricsBase):
    pass

class Biometrics(BiometricsBase):
    uuid: int

class LocationBase(BaseModel):
    latitude: int
    longitude: int
    altitude: int
    heading: int

class LocationCreate(LocationBase):
    pass
    
class Location(LocationBase):
    uuid: int

class UserBase(BaseModel):
    user_id: str

class UserCreate(UserBase):
    pass

class User(UserBase):
    uuid: int

    class Config:
        orm_mode = True