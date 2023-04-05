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
    // e.preventDefault();
    // procedureData.name = e.target["proc-name"].value;
    // procedureData.summary = e.target["proc-summary"].value;
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

  useEffect(() => {
    console.log(newProcedure);
  }, [setNewProcedure])

  return (
    <>
      <SideNav />
      <div className="Page">
        <h1>Procedures</h1>
        <p>Ground Control Panel</p>
        <div className="Container-primary">
          <form onSubmit={() => handleCreate}>
            <h3>Create Procedure</h3>
            <label>Procedure Details</label>
            <input type="text" name="proc-name" placeholder="Name" />
            <input type="text" name="proc-summary" placeholder="Summary" />
            <label>Tasks</label>
            <div className="Task-list" style={{display: "flex", flexDirection: "row", flexWrap: "wrap"}}>
              {newProcedure.taskList.length > 0 ? (
                newProcedure.taskList.map((task, i) => {
                  return (
                    <div key={i} className="Pretty-form">
                      <label>Task {i + 1}</label>
                      <input
                        type="text"
                        name={"task-name" + i}
                        defaultValue={task.name}
                        placeholder="Name"
                      />
                      <input
                        type="text"
                        name={"task-summary" + i}
                        defaultValue={task.summary}
                        placeholder="Summary"
                      />
                      <label>Steps</label>
                      <div className="Step-list">
                        {newProcedure.taskList[i].stepList.length > 0 ? (
                          newProcedure.taskList[i].stepList.map((step, j) => {
                            return (
                              <div key={j} className="Pretty-form">
                                <label>Step {j + 1}</label>
                                <input
                                  type="text"
                                  name={"step-type" + j}
                                  defaultValue={step.type}
                                  placeholder="Type"
                                />
                                <input
                                  type="text"
                                  name={"step-body" + j}
                                  defaultValue={step.body}
                                  placeholder="Body"
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
