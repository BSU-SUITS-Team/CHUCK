import datetime
from fastapi import APIRouter, Request, Response, status
from pydantic import BaseModel
from app.database import connection

router = APIRouter(prefix="/user", tags=["user"])

class User(BaseModel):
    name: str

@router.get("/")
def get_all_users():
    with connection.cursor() as db:
        db.execute("SELECT * FROM users;")
        result = db.fetchall()
        response = []
        for row in result:
            (id, name, createdAt) = row
            response.append({ "id": id, "name": name, "createdAt": createdAt })
        return { "users": response }

@router.get("/{user}")
async def get_user(user: str):
    try:
        user = int(user)
    except Exception:
        return status.HTTP_400_BAD_REQUEST

    with connection.cursor() as db:
        db.execute("SELECT * FROM users WHERE id = %s;", (user,))
        (id, name, createdAt) = db.fetchone()
        if (id and name and createdAt):
            return { "id": id, "name": name, "createdAt": createdAt }
        else:
            return status.HTTP_404_NOT_FOUND

@router.put("/")
async def put_user(user: User):
    with connection.cursor() as db:
        db.execute("INSERT INTO users (name) VALUES (%s) RETURNING *;", (user.name,))
        (id, name, createdAt) = db.fetchone()
        return { "id": id, "name": name, "createdAt": createdAt}
