from fastapi import APIRouter
from pydantic import BaseModel
from app.database import connection

router = APIRouter(prefix="/uia", tags=["uia"])

class UIAStatusRequest(BaseModel):
    o2: bool | None = None
    power: bool | None = None
    comm: bool | None = None

keys = [
    "panel_id",
    "o2",
    "power",
    "comm",
    "createdAt",
    "updatedAt"
]

@router.get("/")
async def get_uia():
    with connection.cursor() as db:
        db.execute("SELECT * FROM uia ORDER BY updatedAt DESC;")
        rows = db.fetchall()
        return [{i: j for i, j in zip(keys, row[1:])} for row in rows]

@router.get("/{panel_id}")
async def get_uia(panel_id):
    with connection.cursor() as db:
        statement = f"SELECT * FROM uia WHERE panel_id = {panel_id} ORDER BY updatedAt DESC LIMIT 1;"
        db.execute(statement)
        rows = db.fetchall()
        return [{i: j for i, j in zip(keys, row[1:])} for row in rows]

@router.post("/{panel_id}")
async def post_uia(panel_id, body: UIAStatusRequest):
    with connection.cursor() as db:
        updates = [("panel_id", panel_id)]
        if body.o2 != None:
            updates.append(("o2", body.o2))
        if body.power != None:
            updates.append(("power_", body.power))
        if body.comm != None:
            updates.append(("comm", body.comm))
        statement = f"INSERT INTO uia ({', '.join([column for (column, _) in updates])}) VALUES ({', '.join([str(value) for (_, value) in updates])}) RETURNING *;"
        db.execute(statement)
        row = db.fetchone()
        connection.commit()
        return {i: j for i, j in zip(keys, row[1:])}