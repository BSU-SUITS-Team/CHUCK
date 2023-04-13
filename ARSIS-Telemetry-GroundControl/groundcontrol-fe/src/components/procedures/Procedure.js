import React from "react";
import Task from "./Task"

const Procedure = (props) => {
  const {name, taskList} = props
  return (
    <div className="Procedure">
      <label>{name}</label>
      {taskList.map((task, i) => {
        return <Task key={i} task={task} i={i} />
      })}
    </div>
  );
};

export default Procedure;
