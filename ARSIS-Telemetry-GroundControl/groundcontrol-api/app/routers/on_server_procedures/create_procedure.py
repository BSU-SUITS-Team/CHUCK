class CreateProcedure:
    def __init__(self, name, summary):
        self.name = name
        self.summary = summary
        self.task_list = []

    def add_task(self, name, summary, stepList):
        task = {"name": name, "summary": summary, "stepList": stepList}
        self.task_list.append(task)
