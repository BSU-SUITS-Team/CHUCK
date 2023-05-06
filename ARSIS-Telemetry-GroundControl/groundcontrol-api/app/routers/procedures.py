from fastapi import APIRouter
from app.routers.on_server_procedures.create_procedure import CreateProcedure

from .on_server_procedures.mock_procedure import mock_procedure

router = APIRouter(prefix="/procedures", tags=["procedures"])

procedure_list = [mock_procedure]
in_mem_procedures = {p.get_name(): p for p in procedure_list}


@router.get("/")
async def procedures():
    return in_mem_procedures


@router.get("/{name}")
def procedure(name: str):
    res = in_mem_procedures.get(name, None)
    if res is not None:
        return res
    return {"name": "Not found", "taskList": []}


@router.patch("/")
def procedure(incoming_procedure: dict):
    proc_to_update = in_mem_procedures.get(incoming_procedure["name"], None)
    if proc_to_update is None:
        return {"error": f"Procedure with name: {incoming_procedure['name']} not found"}
    updated_proc = CreateProcedure(
        incoming_procedure["name"], incoming_procedure["summary"]
    )
    for task in incoming_procedure["taskList"]:
        updated_proc.add_task(task["name"], task["summary"], task["stepList"])
    in_mem_procedures[incoming_procedure["name"]] = updated_proc.to_dict()
    return {"message": "Procedure successfully updated"}


@router.delete("/{name}")
def procedure(name: str):
    proc_to_delete = in_mem_procedures.get(name, None)
    if proc_to_delete is None:
        return {"error": f"Procedure with name: {name} not found"}
    in_mem_procedures.pop(name)
    return {"message": "Procedure successfully deleted"}


@router.post("/")
def procedure(new_procedure: dict):
    procedure = CreateProcedure(new_procedure["name"], new_procedure["summary"])
    for task in new_procedure["taskList"]:
        procedure.add_task(task["name"], task["summary"], task["stepList"])
    in_mem_procedures[procedure.get_name()] = procedure.to_dict()
    return {"message": "Procedure successfully created"}
