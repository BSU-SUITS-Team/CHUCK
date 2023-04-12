from fastapi import APIRouter
from app.routers.on_server_procedures.create_procedure import CreateProcedure

from .on_server_procedures.mock_procedure import MockProcedure

router = APIRouter(prefix="/procedures", tags=["procedures"])

mock_procedure = MockProcedure()

in_mem_procedures = {mock_procedure.get_name(): { "summary": mock_procedure.get_summary(), "taskList": mock_procedure.get_task_list_encoded()}}

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
def procedure(incoming_procedure: dict):
    proc_to_update = in_mem_procedures.get(incoming_procedure["name"], None)
    if proc_to_update is None:
        return { "message": f"Procedure with name: {incoming_procedure['name']} not found"}
    updated_proc = CreateProcedure(incoming_procedure["name"], incoming_procedure["summary"])
    for task in incoming_procedure["taskList"]:
        updated_proc.add_task(task["name"], task["summary"], task["stepList"])
    in_mem_procedures[incoming_procedure["name"]] = updated_proc.to_json()
    return { "message": "Procedure successfully updated"}

@router.post("/")
def procedure(new_procedure: dict):
    procedure = CreateProcedure(new_procedure["name"], new_procedure["summary"])
    for task in new_procedure["taskList"]:
        procedure.add_task(task["name"], task["summary"], task["stepList"])
    in_mem_procedures[procedure.get_name()] = procedure.to_json()
    return { "message": "Procedure successfully created"}