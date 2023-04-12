import React, { useState } from "react";

const UpdateProcedureMenu = (props) => {
  const [proc, setProc] = useState(props.selectedProc);

  const handleSubmit = (e) => {
    e.preventDefault();
    let procedureData = { ...proc };
    procedureData.summary = e.target["proc-summary"].value;
    procedureData.taskList.map((task, i) => {
      task.name = e.target["task-name" + i].value;
      task.summary = e.target["task-summary" + i].value;
      task.stepList.map((step, j) => {
        step.type = e.target["step-type" + i + j].value;
        step.body = e.target["step-type" + i + j].value;
        step.nextTask =
          j === task.stepList.length - 1 &&
          i !== procedureData.taskList.length - 1
            ? { procedure: procedureData.name, task: i + 1 }
            : null;
      });
    });
    fetch("http://localhost:8181/procedures/", {
      method: "PATCH",
      mode: "cors",
      headers: {
        "Content-Type": "application/json",
      },
      body: JSON.stringify(procedureData),
    }).catch((err) => {
      if (err) {
        console.log(err);
      }
    });
    props.onChangeIsEditing(false);
  };

  const handleAddTask = () => {
    let procedureData = { ...proc };
    procedureData.taskList.push({
      name: "",
      summary: "",
      stepList: [{ type: "", body: "", nextTask: "" }],
    });
    setProc(procedureData);
  };

  const handleRemoveTask = () => {
    let procedureData = { ...proc };
    procedureData.taskList.pop();
    setProc(procedureData);
  };

  const handleAddStep = (index) => {
    let procedureData = { ...proc };
    procedureData.taskList[index].stepList.push({
      type: "",
      body: "",
      nextTask: "",
    });
    setProc(procedureData);
  };
  const handleRemoveStep = (index) => {
    let procedureData = { ...proc };
    procedureData.taskList[index].stepList.pop();
    setProc(procedureData);
  };

  const handleDelete = () => {
    fetch("http://localhost:8181/procedures/", {
      method: "DELETE",
      mode: "cors",
      headers: {
        "Content-Type": "application/json",
      },
      body: JSON.stringify({ name: proc.name }),
    }).catch((err) => {
      if (err) {
        console.log(err);
      }
    });
  };

  return (
    <div className="Container-primary">
      <form onSubmit={handleSubmit}>
        <h3>Update Procedure Menu</h3>
        <label>Procedure Details</label>
        <label>Editing Procedure: {proc.name}</label>
        <textarea
          type="text"
          style={{ maxWidth: "300px" }}
          name="proc-summary"
          placeholder="Summary"
          defaultValue={proc.summary}
        />
        <label>Tasks</label>
        <div className="Wrapped-list">
          {proc.taskList.length > 0 ? (
            proc.taskList.map((task, i) => {
              return (
                <div key={i} className="Task">
                  <label>Task {i + 1}</label>
                  <input
                    type="text"
                    name={"task-name" + i}
                    defaultValue={task.name}
                    placeholder="Name"
                  />
                  <textarea
                    name={"task-summary" + i}
                    defaultValue={task.summary}
                    placeholder="Summary"
                  />
                  <label>Steps</label>
                  <div>
                    {proc.taskList[i].stepList.length > 0 ? (
                      proc.taskList[i].stepList.map((step, j) => {
                        return (
                          <div key={j} className="Step">
                            <label>Step {j + 1}</label>
                            <input
                              type="text"
                              name={"step-type" + i + j}
                              defaultValue={step.type}
                              placeholder="Type"
                            />
                            <textarea
                              name={"step-body" + i + j}
                              defaultValue={step.body}
                              placeholder="Body"
                              row="5"
                              cols="20"
                            />
                          </div>
                        );
                      })
                    ) : (
                      <p>No Steps</p>
                    )}
                    <div className="Button-group">
                      <button type="button" onClick={() => handleAddStep(i)}>
                        Add Step
                      </button>
                      <button type="button" onClick={() => handleRemoveStep(i)}>
                        Remove Step
                      </button>
                    </div>
                  </div>
                </div>
              );
            })
          ) : (
            <p>No tasks</p>
          )}
        </div>
        <div className="Button-group">
          <button type="button" onClick={handleAddTask}>
            Add Task
          </button>
          <button type="button" onClick={handleRemoveTask}>
            Remove Task
          </button>
          <button type="button" onClick={handleDelete}>
            Delete Procedure
          </button>
          <button type="submit">Save Changes</button>
        </div>
      </form>
    </div>
  );
};

export default UpdateProcedureMenu;
