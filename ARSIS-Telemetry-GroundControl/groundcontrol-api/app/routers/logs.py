from fastapi import APIRouter, Response, status
from app.database import connection
from pydantic import BaseModel

class Log(BaseModel):
    data: str

router = APIRouter(prefix="/logs", tags=["logs"])

@router.get("/", status_code=200)
def get_logs(limit: int=10):
    with connection.cursor() as db:
        db.execute(f"SELECT * FROM logs LIMIT {limit};")
        result = db.fetchall()
        response = []
        for row in result:
            (id, data, createdAt) = row
            response.append({"id": id, "data": data, "createdAt": createdAt})
        return {"logs": response}

@router.get("/{uuid}", status_code=200)
def get_log(uuid: int, res: Response):
    with connection.cursor() as db:
        db.execute(f"SELECT * FROM logs WHERE uuid='{uuid}';")
        returned = db.fetchone()
        if returned is not None:
            (id, data, createdAt) = returned
            return {"id": id, "data": data, "createdAt": createdAt}
        else:
            res.status = status.HTTP_404_NOT_FOUND
            return {f"Log for uuid: {uuid} not found"}

@router.put("/", status_code=201)
def create_log(log: Log):
    with connection.cursor() as db:
        try:
            to_insert = log.data
            to_replace = '\''
            replacer = '\"'
            query = f"INSERT INTO logs (createdAt, data) VALUES (DEFAULT, \'{to_insert.replace(to_replace, replacer)}\') RETURNING *;"
            db.execute(query)
            (uuid, data, createdAt) = db.fetchone()
            return {"uuid": uuid, "data": data, "createdAt": createdAt}
        except:
            connection.rollback()
        finally:
            connection.commit()