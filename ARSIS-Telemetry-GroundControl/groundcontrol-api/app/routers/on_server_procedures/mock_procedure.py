import base64

from .create_procedure import CreateProcedure

mock_procedure_name = "Mock Procedure"
mock_procedure_summary = "This is a 100% fake procedure that only exists to make sure we can get data through the system"
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

task_2_list = [
    {
        "type": "text",
        "body": "2nd step of mock procedure",
        "nextTask": {"procedure": "Idle Procedure", "step": 0},
    },
]
task_2_name = "Name of Task"
task_2_summary = "Summary of what task is meant to do"
mock_procedure = CreateProcedure(mock_procedure_name, mock_procedure_summary)
mock_procedure.add_task(task_1_name, task_1_summary, task_1_list)
mock_procedure.add_task(task_2_name, task_2_summary, task_2_list)
# class MockProcedure:
#     def __init__(self):
#         self.name = "Mock Procedure"
#         self.summary = "This is a 100% fake procedure that only exists to make sure we can get data through the system"
#
#         suits_image = base64.b64encode(
#             open("app/routers/on_server_procedures/NASASUITS-logo.png", "rb").read()
#         )
#         task_1_list = [
#             {"type": "image", "body": suits_image, "nextTask": None},
#             {
#                 "type": "text",
#                 "body": "do this next thing and if it works goto otherwise",
#                 "nextTask": {"procedure": "Mock Procedure", "task": 2},
#             },
#         ]
#         task_1 = {
#             "name": "Name of Task",
#             "summary": "Summary of what task is meant to do",
#             "stepList": task_1_list,
#         }
#
#         task_2_list = [
#             {
#                 "type": "text",
#                 "body": "2nd step of mock procedure",
#                 "nextTask": {"procedure": "Idle Procedure", "step": 0},
#             },
#         ]
#         task_2 = {
#             "name": "Name of Task",
#             "summary": "Summary of what task is meant to do",
#             "stepList": task_2_list,
#         }
#         self.taskList = [task_1, task_2]
#
#     def get_name(self):
#         return self.name
#
#     def get_summary(self):
#         return self.summary
#
#     def get_task_list(self):
#         return self.taskList
#
#     def get_task_list_encoded(self):
#         return self.taskList
#
#     def get_dict(self):
#         return {"name": self.name, "summary": self.summary, "taskList": self.taskList}
