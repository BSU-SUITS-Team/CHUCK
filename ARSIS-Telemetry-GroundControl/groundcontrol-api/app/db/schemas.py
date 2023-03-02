from pydantic import BaseModel

class LogBase(BaseModel):
    data: str

class LogCreate(LogBase):
    pass
    
class Log(LogBase):
    uuid: int

    class Config:
        orm_mode = True