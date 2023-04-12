import React from "react";
import Task from "./Task"

const Procedure = (props) => {
  return (
    <div className="Procedure">
      <label>{props.name}</label>
      {props.taskList.map((task, j) => {
        return <Task key={j} name={task.name}/>
      })}
    </div>
  );
};

export default Procedure;
