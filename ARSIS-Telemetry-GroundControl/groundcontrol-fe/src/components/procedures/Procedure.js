import React, { useContext } from "react";
import Task from "./Task";

import { ProcedureContext } from "../../pages/Procedures";

const Procedure = () => {
  const { func, tabs } = useContext(ProcedureContext);
  const [procedure, setProcedure] = func;
  const [tab, setTab] = tabs;

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

  return (
    <div className="Procedure">
      {tab === 1 ? (
        <input
          type="text"
          style={{ maxWidth: "300px" }}
          name="proc-name"
          placeholder="Name"
        />
      ) : (
        <h2>Editing Procedure: {procedure.name}</h2>
      )}
      <textarea
        type="text"
        style={{ maxWidth: "300px" }}
        name="proc-summary"
        placeholder="Summary"
      />
      <h3>Tasks</h3>
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
      </div>
    </div>
  );
};

export default Procedure;
