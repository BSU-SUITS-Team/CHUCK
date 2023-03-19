from fastapi import APIRouter
from pydantic import BaseModel
from app.database import connection

router = APIRouter(prefix="/uia", tags=["uia"])

class UIAStatusRequest(BaseModel):
    o2: bool | None = None
    power: bool | None = None
    comm: bool | None = None

@router.get("/")
async def get_uia():
    with connection.cursor() as db:
        db.execute("SELECT * FROM uia WHERE id = 1;")
        (id, o2, power, comm, updatedAt) = db.fetchone()
        response = {
            "o2": o2,
            "power": power,
            "comm": comm,
            "updatedAt": updatedAt
        }
        return response

@router.post("/")
async def post_uia(body: UIAStatusRequest):
    with connection.cursor() as db:
        if body.o2 != None:
            db.execute("UPDATE uia SET o2 = %s, updatedAt = now() WHERE id = 1;", (body.o2,))

        if body.power != None:
            db.execute("UPDATE uia SET power_ = %s, updatedAt = now() WHERE id = 1;", (body.power,))

        if body.comm != None:
            db.execute("UPDATE uia SET comm = %s, updatedAt = now() WHERE id = 1;", (body.comm,))

        db.execute("SELECT * FROM uia WHERE id = 1;")
        (id, o2, power, comm, updatedAt) = db.fetchone()

        response = {
            "o2": o2,
            "power": power,
            "comm": comm,
            "updatedAt": updatedAt
        }
        return response