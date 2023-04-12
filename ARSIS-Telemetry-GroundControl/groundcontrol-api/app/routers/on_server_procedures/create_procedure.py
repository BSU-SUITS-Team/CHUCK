class CreateProcedure:
    def __init__(self, name, summary):
        self.name = name
        self.summary = summary
        self.task_list = []

    def add_task(self, name, summary, step_list):
        task = {"name": name, "summary": summary, "stepList": step_list}
        self.task_list.append(task)

    def get_name(self):
        return self.name
    
    def get_summary(self):
        return self.summary

    def get_task_list(self):
        return self.task_list

    def get_task_list_encoded(self):
        return self.task_list
    
    def to_json(self):
        return {"summary": self.summary, "taskList": self.get_task_list_encoded()}