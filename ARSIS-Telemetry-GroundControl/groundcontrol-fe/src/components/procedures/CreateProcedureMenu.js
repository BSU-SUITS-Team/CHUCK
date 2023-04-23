import React, { useContext } from "react";
import { ProcedureContext } from "../../pages/Procedures";
import Procedure from "./Procedure"

const CreateProcedureMenu = () => {
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
        step.body = step.type === "image" ? e.target["step-body" + i + j].src : e.target["step-body" + i + j].value;
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

  return (
    <div className="Container-primary">
      <form onSubmit={handleCreate}>
        <h3>Create Procedure</h3>
        <label>Procedure Details</label>
          <Procedure />
          <button type="submit">Create</button>
      </form>
    </div>
  );
};

export default CreateProcedureMenu;
