import React, { useEffect, useState } from "react";

const UpdateProcedureMenu = () => {
  const [procedures, setProcedures] = useState([{ "name": "Fake Proc", "taskList": [{ 'name': 'Fake Task list' }] }]);

  const handleRefreshProcedures = async () => {
    const proceduresList = [];
    const response = await fetch("http://localhost:8181/procedures/");
    const data = await response.json();
    for (const [key, value] of Object.entries(data)) {
      proceduresList.push({ "name": key, "taskList": [...value] });
    }
    setProcedures(proceduresList);
  };

  const handleEditProcedure = () => {
    console.log("Edit Procedure")
  }

  useEffect(() => {
    handleRefreshProcedures()
  }, [])

  const handleSubmit = (e) => {};

  return (
    <div className="Container-primary">
      <form onSubmit={handleSubmit}>
        <h3>View Procedure Menu</h3>
        <div>
          {procedures.length > 0 ? (
            procedures.map((proc, i) => {
              return (
                <div key={i}>
                  <label>{proc.name}</label>
                  {proc.taskList.map((task, j) => {
                    return (
                      <div key={j}>
                        {task.name}
                      </div>
                    )
                  })}
                  <button type="button" onClick={handleEditProcedure}>Edit Procedure</button>
                </div>
              );
            })
          ) : (
            <div>No Procedures</div>
          )}
        </div>
        <button type="button" onClick={handleRefreshProcedures}>
          Refresh
        </button>
      </form>
    </div>
  );
};

export default UpdateProcedureMenu;
