from pydantic import BaseModel

class BiometricBase(BaseModel):
    data: str

class BiometricCreate(BiometricBase):
    pass

class Biometric(BiometricBase):
    uuid: int

class LocationBase(BaseModel):
    data: str

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