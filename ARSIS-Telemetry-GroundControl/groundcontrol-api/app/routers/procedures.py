from fastapi import APIRouter
from pydantic import BaseModel
from app.routers.on_server_procedures.create_procedure import CreateProcedure

from .on_server_procedures.mock_procedure import MockProcedure

router = APIRouter(prefix="/procedures", tags=["procedures"])

mock_procedure = MockProcedure()

in_mem_procedures = {mock_procedure.get_name(): mock_procedure.get_task_list_encoded()}

@router.get("/")
async def procedures():
    return in_mem_procedures


@router.get("/{name}")
def procedure(name: str):
    res = in_mem_procedures.get(name, None)
    if res is not None:
        return {
            "name": name,
            "taskList": res,
        }
    return {"name": "Not found", "taskList": []}

@router.patch("/")
def procedure(updated_procedure: dict):

    return { "procedure": { "name": procedure.name, "summary": procedure.summary, "taskList": procedure.task_list}}

@router.post("/")
def procedure(new_procedure: dict):
    procedure = CreateProcedure(new_procedure["name"], new_procedure["summary"])
    for task in new_procedure["taskList"]:
        procedure.add_task(task["name"], task["summary"], task["stepList"])
    in_mem_procedures["procedure.get_name()"] = procedure.get_task_list_encoded()
    return { "procedure": { "name": procedure.get_name(), "taskList": procedure.get_task_list_encoded() }}