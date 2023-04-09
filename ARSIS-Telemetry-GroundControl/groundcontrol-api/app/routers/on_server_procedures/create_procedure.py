class CreateProcedure:
    def __init__(self, name, summary):
        self.name = name
        self.summary = summary
        self.task_list = []

    def add_task(self, name, summary, stepList):
        task = {"name": name, "summary": summary, "stepList": stepList}
        self.task_list.append(task)

    def get_name(self):
        return self.name

    def get_task_list(self):
        return self.task_list

    def get_task_list_encoded(self):
        return self.task_list
