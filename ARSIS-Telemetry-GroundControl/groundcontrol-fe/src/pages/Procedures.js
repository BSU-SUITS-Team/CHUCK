import React, { useState } from "react";
import SideNav from "../components/SideNav";

const Procedures = () => {
  const [newProcedure, setNewProcedure] = useState({});
  const [tasks, setTasks] = useState([{ name: "", summary: "", stepList: "" }]);
  const [steps, setSteps] = useState([{ type: "", body: "", nextTask: "" }]);

  const handleCreate = () => {
    console.log("created procedure");
  };

  const handleAddTask = () => {
    setTasks([...tasks, { name: "", summary: "", stepList: "" }]);
  };

  const handleAddStep = () => { 
    console.log("added step");
  }

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
            <input type="text" name="proc-name" placeholder="Name" />
            <input type="text" name="proc-summary" placeholder="Summary" />
            <label>Tasks</label>
            <div>
              {tasks.length > 0 ? (
                tasks.map((task, i) => {
                  return (
                    <div key={i} className='Pretty-form'>
                      <label>Task {i + 1}</label>
                      <input
                        type="text"
                        name="task-name"
                        placeholder="Name"
                        value={task.name}
                      />
                      <input
                        type="text"
                        name="task-summary"
                        placeholder="Summary"
                        value={task.summary}
                      />
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
              <button type="submit">Create</button>
            </div>
          </form>
        </div>
      </div>
    </>
  );
};

export default Procedures;
