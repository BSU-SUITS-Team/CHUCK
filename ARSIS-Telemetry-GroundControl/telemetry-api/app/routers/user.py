from fastapi import APIRouter, status
from pydantic import BaseModel
from app.database import connection
from psycopg2 import errors

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
            response.append({"id": id, "name": name, "createdAt": createdAt})
        return {"users": response}


@router.get("/{user_id}")
async def get_user(user_id: int):
    with connection.cursor() as db:
        db.execute("SELECT * FROM users WHERE id = %s;", (user_id,))
        (id, name, createdAt) = db.fetchone()
        if id and name and createdAt:
            return {"id": id, "name": name, "createdAt": createdAt}
        else:
            return status.HTTP_404_NOT_FOUND
        connection.commit()


@router.put("/")
async def put_user(user: User):
    with connection.cursor() as db:
        try:
            db.execute("INSERT INTO users (name) VALUES (%s) RETURNING *;", (user.name,))
            (id, name, createdAt) = db.fetchone()
            return {"id": id, "name": name, "createdAt": createdAt}
        except errors.UniqueViolation as err:
            connection.rollback()
        finally:
            connection.commit()
