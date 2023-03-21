from fastapi import APIRouter
from pydantic import BaseModel
from app.database import connection

router = APIRouter(prefix="/uia", tags=["uia"])

class UIAStatusRequest(BaseModel):
    o2: bool | None = None
    power: bool | None = None
    comm: bool | None = None

keys = [
    "id",
    "power",
    "comm",
    "updatedAt"
]

@router.get("/")
async def get_uia():
    with connection.cursor() as db:
        db.execute("SELECT * FROM uia WHERE id = 1;")
        row = db.fetchone()
        return {i: j for i, j in zip(keys, row[1:])}

@router.post("/")
async def post_uia(body: UIAStatusRequest):
    with connection.cursor() as db:
        o2_update = f"o2 = {body.o2}, " if body.o2 != None else ""
        power_update = f"power_ = {body.power}, " if body.power != None else ""
        comm_update =f"comm = {body.comm}, " if body.comm != None else ""
        values = f"{o2_update}{power_update}{comm_update}"
        statement = f"UPDATE uia SET {values} updatedAt = now() WHERE id = 1 RETURNING *;"
        db.execute(statement)
        row = db.fetchone()
        connection.commit()
        return {i: j for i, j in zip(keys, row[1:])}