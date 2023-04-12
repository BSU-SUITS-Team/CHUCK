import React from "react";
import Task from "./Task"

const Procedure = (props) => {
  const handleEditProcedure = () => {
    console.log("Edit Procedure: " + props.name)
  }
  return (
    <div className="Procedure">
      <label>{props.name}</label>
      {props.taskList.map((task, j) => {
        return <Task key={j} name={task.name}/>
      })}
      <button type="button" onClick={handleEditProcedure}>Edit Procedure</button>
    </div>
  );
};

export default Procedure;
