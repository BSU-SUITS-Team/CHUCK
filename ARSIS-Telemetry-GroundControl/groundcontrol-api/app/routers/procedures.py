from app.datastore import ds
from app.event import Event
from app.routers.on_server_procedures.create_procedure import CreateProcedure
from fastapi import APIRouter
import asyncio
import os
import base64
import yaml

router = APIRouter(prefix="/procedures", tags=["procedures"])


def load_yml_procedure(yml_file):
    with open(yml_file, 'r') as file:
        data = yaml.safe_load(file)

    def encode_files(obj):
        if isinstance(obj, dict):
            for key, value in obj.items():
                if isinstance(value, str) and os.path.isfile('./app/routers/procedures/' + value):
                    with open('./app/routers/procedures/' + value, 'rb') as file:
                        encoded_content = base64.b64encode(
                            file.read()).decode('utf-8')
                        obj[key] = encoded_content
                elif isinstance(value, (dict, list)):
                    encode_files(value)
        elif isinstance(obj, list):
            for item in obj:
                if isinstance(item, (dict, list)):
                    encode_files(item)

    encode_files(data)
    return data


procedure_list = [load_yml_procedure('./app/routers/procedures/' + procedure)
                  for procedure in os.listdir('./app/routers/procedures/') if procedure.endswith(".yml")]
in_mem_procedures = {p["name"]: p for p in procedure_list}


async def add_procedure_to_ds(procedure):
    procedure_event = Event.create_event(
        "procedure", procedure, upsert_key=procedure['name'])
    await ds.add_event("procedure", procedure_event)


ts = []
for p in procedure_list:
    t = asyncio.create_task(add_procedure_to_ds(p))
    ts.append(t)
asyncio.gather(*ts)


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
def procedure_PATCH(incoming_procedure: dict):
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
def procedure_DELETE(name: str):
    proc_to_delete = in_mem_procedures.get(name, None)
    if proc_to_delete is None:
        return {"error": f"Procedure with name: {name} not found"}
    in_mem_procedures.pop(name)
    return {"message": "Procedure successfully deleted"}


@router.post("/")
def procedure_POST(new_procedure: dict):
    print(new_procedure)
    in_mem_procedures[new_procedure['name']] = new_procedure
    asyncio.run(add_procedure_to_ds(new_procedure))
    return {"message": "Procedure successfully created"}
