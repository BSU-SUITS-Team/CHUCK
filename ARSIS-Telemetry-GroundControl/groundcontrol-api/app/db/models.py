from sqlalchemy import JSON, Column, ForeignKey, Serial, TIMESTAMP

from .database import Base


class Log(Base):
    __tablename__ = "logs"

    uuid = Column(Serial, primary_key=True, index=True)
    timeIn = Column(TIMESTAMP, index=True)
    log = Column(JSON)