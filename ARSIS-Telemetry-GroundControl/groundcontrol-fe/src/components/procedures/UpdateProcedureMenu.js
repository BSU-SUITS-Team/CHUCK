import React, { useContext, useState } from "react";
import Task from "./Task";
import { ProcedureContext } from "../../pages/Procedures";
import Procedure from "./Procedure";

const UpdateProcedureMenu = () => {
  const { func, tabs } = useContext(ProcedureContext);
  const [procedure, setProcedure] = func;
  const [tab, setTab] = tabs;

  const handleSubmit = async (e) => {
    e.preventDefault();
    let procedureData = { ...procedure };
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
    const result = await fetch("http://localhost:8181/procedures/", {
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
    const data = await result.json();
    alert(data.message);
    setTab();
  };

  const handleDelete = async () => {
    const result = await fetch(
      `http://localhost:8181/procedures/${procedure.name}`,
      {
        method: "DELETE",
        mode: "cors",
      }
    ).catch((err) => {
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
      <form onSubmit={handleSubmit}>
        <h3>Update Procedure Menu</h3>
        <label>Procedure Details</label>
        <Procedure />
        <div className="Button-group">
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
