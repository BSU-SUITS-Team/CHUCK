import base64


class MockProcedure:
    def __init__(self):
        self.name = "Mock Procedure"
        self.summary = "This is a 100% fake procedure that only exists to make sure we can get data through the system"

        suits_image = base64.b64encode(
            open("app/routers/on_server_procedures/NASASUITS-logo.png", "rb").read()
        )
        task_1_list = [
            {"type": "image", "body": suits_image, "next_step": ""},
            {
                "type": "text",
                "body": "do this next thing and if it works goto otherwise",
                "next_steps": {"procedure": "Mock Procedure", "step": 2},
            },
        ]
        task_1 = {
            "name": "Name of Task",
            "summary": "Summary of what task is meant to do",
            "stepList": task_1_list,
        }

        task_2_list = [
            {
                "type": "text",
                "body": "2nd step of mock procedure",
                # "next_steps": {"procedure": "Idle Procedure", "step": 0},
            },
        ]
        task_2 = {
            "name": "Name of Task",
            "summary": "Summary of what task is meant to do",
            "stepList": task_2_list,
        }
        self.task_list = [task_1, task_2]

    def get_name(self):
        return self.name

    def get_task_list(self):
        return self.task_list

    def get_task_list_encoded(self):
        return self.task_list
