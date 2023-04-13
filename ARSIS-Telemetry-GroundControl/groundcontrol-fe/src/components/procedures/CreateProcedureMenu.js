import React, { useContext } from "react";
import { ProcedureContext } from "../../pages/Procedures";
import Task from "./Task"

const CreateProcedureMenu = (props) => {
  const { func, tabs } = useContext(ProcedureContext)
  const [procedure, setProcedure] = func
  const [tab, setTab] = tabs

  const handleCreate = async (e) => {
    e.preventDefault();
    let procedureData = { ...procedure };
    procedureData.name = e.target["proc-name"].value;
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
            ? { procedure: e.target["proc-name"].value, task: i + 1 }
            : null;
      });
    });
    const result = await fetch("http://localhost:8181/procedures/", {
      method: "POST",
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
    const data = await result.json();
    alert(data.message);
    setTab();
  };

  const handleAddTask = () => {
    let procedureData = { ...procedure };
    procedureData.taskList.push({
      name: "",
      summary: "",
      stepList: [{ type: "", body: "", nextTask: "" }],
    });
    setProcedure(procedureData);
  };

  const handleRemoveTask = () => {
    let procedureData = { ...procedure };
    procedureData.taskList.pop();
    setProcedure(procedureData);
  };

  const handleAddStep = (index) => {
    let procedureData = { ...procedure };
    procedureData.taskList[index].stepList.push({
      type: "",
      body: "",
      nextTask: "",
    });
    setProcedure(procedureData);
  };
  const handleRemoveStep = (index) => {
    let procedureData = { ...procedure };
    procedureData.taskList[index].stepList.pop();
    setProcedure(procedureData);
  };
  return (
    <div className="Container-primary">
      <form onSubmit={handleCreate}>
        <h3>Create Procedure</h3>
        <label>Procedure Details</label>
        <input
          type="text"
          style={{ maxWidth: "300px" }}
          name="proc-name"
          placeholder="Name"
        />
        <textarea
          type="text"
          style={{ maxWidth: "300px" }}
          name="proc-summary"
          placeholder="Summary"
        />
        <label>Tasks</label>
        <div className="Wrapped-list">
          {procedure.taskList.length > 0 ? (
            procedure.taskList.map((task, i) => {
              return <Task key={i} task={task} i={i} />;
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
  );
};

export default CreateProcedureMenu;
