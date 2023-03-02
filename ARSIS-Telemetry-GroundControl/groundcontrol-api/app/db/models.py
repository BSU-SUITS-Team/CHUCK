from sqlalchemy import JSON, Column, INTEGER, TIMESTAMP

from .database import Base


class Log(Base):
    __tablename__ = "logs"

    uuid = Column(INTEGER, primary_key=True, index=True)
    timein = Column(TIMESTAMP, index=True)
    data = Column(JSON)