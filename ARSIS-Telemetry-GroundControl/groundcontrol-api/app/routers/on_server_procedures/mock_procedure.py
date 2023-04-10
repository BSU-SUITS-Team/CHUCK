import base64
from .create_procedure import CreateProcedure

mock_procedure_name = "Mock Procedure"
mock_procedure_summary = "This is a 100% fake procedure that only exists to make sure we can get data through the system"
mock_procedure = CreateProcedure(mock_procedure_name, mock_procedure_summary)
suits_image = base64.b64encode(
    open("app/routers/on_server_procedures/NASASUITS-logo.png", "rb").read()
)
task_1_list = [
    {"type": "image", "body": suits_image, "nextTask": None},
    {
        "type": "text",
        "body": "do this next thing and if it works goto otherwise",
        "nextTask": {"procedure": "Mock Procedure", "task": 2},
    },
]
task_1_name = "Name of Task"
task_1_summary = "Summary of what task is meant to do"
mock_procedure.add_task(task_1_name, task_1_summary, task_1_list)

task_2_list = [
    {
        "type": "text",
        "body": "2nd step of mock procedure",
        "nextTask": {"procedure": "Idle Procedure", "step": 0},
    },
]
task_2_name = "Name of Task"
task_2_summary = "Summary of what task is meant to do"

mock_procedure.add_task(task_2_name, task_2_summary, task_2_list)
