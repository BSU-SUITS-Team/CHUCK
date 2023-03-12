from pydantic import BaseModel

class BiometricBase(BaseModel):
    user_id: str
    o2: int
    battery: int
    bpm: int

class BiometricCreate(BiometricBase):
    pass

class Biometric(BiometricBase):
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