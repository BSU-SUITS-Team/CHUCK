from pydantic import BaseModel


class BiometricsBase(BaseModel):
    o2: int = 0
    battery: int = 0
    bpm: int = 0


class Biometrics(BiometricsBase):
    uuid: int


class LocationBase(BaseModel):
    latitude: int = 0
    longitude: int = 0
    altitude: int = 0
    heading: int = 0


class Location(LocationBase):
    uuid: int


class UserBase(BaseModel):
    name: str


class User(UserBase):
    uuid: int

    class Config:
        orm_mode = True
