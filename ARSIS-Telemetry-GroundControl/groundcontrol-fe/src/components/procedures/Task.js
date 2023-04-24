import React, { useContext } from "react";
import { ProcedureContext } from "../../pages/Procedures";
import Step from "./Step"

const Task = (props) => {
  const { task, i } = props

  const { func } = useContext(ProcedureContext)
  const [procedure, setProcedure] = func

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
    <div className="Task">
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
        {procedure.taskList[i].stepList.length > 0 ? (
          procedure.taskList[i].stepList.map((step, j) => {
            return <Step key={i + "" + j} step={step} i={i} j={j} />;
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
};

export default Task;
