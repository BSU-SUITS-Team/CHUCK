import React, { useEffect, useState } from "react";
import SideNav from "../components/SideNav";

const Procedures = () => {
  const defaultProc = {
    name: "",
    summary: "",
    taskList: [
      {
        name: "",
        summary: "",
        stepList: [{ type: "", body: "", nextTask: "" }],
      },
    ],
  };
  const [newProcedure, setNewProcedure] = useState(defaultProc);

  const handleCreate = (e) => {
    e.preventDefault();
    let procedureData = { ...newProcedure };
    procedureData.name = e.target["proc-name"].value;
    procedureData.summary = e.target["proc-summary"].value;
    procedureData.taskList.map((task, i) => {
      task.name = e.target["task-name" + i].value;
      task.summary = e.target["task-summary" + i].value;
      task.stepList.map((step, j) => {
        step.type = e.target["step-type" + i + j].value;
        step.body = e.target["step-type" + i + j].value;
        step.nextTask =
          j === task.stepList.length - 1
            ? { procedure: e.target["proc-name"].value, task: i + 1 }
            : null;
      });
    });
    fetch("http://localhost:8181/procedures/", {
      method: "POST",
      mode: "cors",
      headers: {
        "Content-Type": "application/json",
      },
      body: JSON.stringify(procedureData),
    })
  };

  const handleAddTask = () => {
    let procedureData = { ...newProcedure };
    procedureData.taskList.push({
      name: "",
      summary: "",
      stepList: [{ type: "", body: "", nextTask: "" }],
    });
    setNewProcedure(procedureData);
  };

  const handleRemoveTask = () => {
    let procedureData = { ...newProcedure };
    procedureData.taskList.pop();
    setNewProcedure(procedureData);
  };

  const handleAddStep = (index) => {
    let procedureData = { ...newProcedure };
    procedureData.taskList[index].stepList.push({
      type: "",
      body: "",
      nextTask: "",
    });
    setNewProcedure(procedureData);
  };
  const handleRemoveStep = (index) => {
    let procedureData = { ...newProcedure };
    procedureData.taskList[index].stepList.pop();
    setNewProcedure(procedureData);
  };

  return (
    <>
      <SideNav />
      <div className="Page">
        <h1>Procedures</h1>
        <p>Ground Control Panel</p>
        <div className="Container-primary">
          <form onSubmit={handleCreate}>
            <h3>Create Procedure</h3>
            <label>Procedure Details</label>
            <input type="text" style={{maxWidth: "300px"}} name="proc-name" placeholder="Name" />
            <textarea type="text" style={{maxWidth: "300px"}} name="proc-summary" placeholder="Summary" />
            <label>Tasks</label>
            <div
              className="Task-list"
              style={{
                display: "flex",
                flexDirection: "row",
                flexWrap: "wrap",
              }}
            >
              {newProcedure.taskList.length > 0 ? (
                newProcedure.taskList.map((task, i) => {
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
                      <div className="Task-list">
                        {newProcedure.taskList[i].stepList.length > 0 ? (
                          newProcedure.taskList[i].stepList.map((step, j) => {
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
                          <button
                            type="button"
                            onClick={() => handleAddStep(i)}
                          >
                            Add Step
                          </button>
                          <button
                            type="button"
                            onClick={() => handleRemoveStep(i)}
                          >
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
              <button type="submit">Create</button>
            </div>
          </form>
        </div>
      </div>
    </>
  );
};

export default Procedures;
