import React, { useEffect, useState } from "react";
import Procedure from "./Procedure";

const ViewProceduresMenu = (props) => {
  const [procedures, setProcedures] = useState([
    { name: "Fake Proc", summary: "Fake summary", taskList: [{ name: "Fake Task list" }] },
  ]);

  const handleRefreshProcedures = async () => {
    const proceduresList = [];
    const response = await fetch("http://localhost:8181/procedures/");
    const data = await response.json();
    for (const [key, value] of Object.entries(data)) {
      proceduresList.push({ name: key, summary: value.summary, taskList: [...value.taskList] });
    }
    setProcedures(proceduresList);
  };

  useEffect(() => {
    handleRefreshProcedures();
  }, []);

  const handleEditProcedure = (proc) => {
    props.onChangeSelectedProc(proc);
    props.onChangeIsEditing(true);
  };

  return (
    <div className="Container-primary">
      <form>
        <h3>All Procedures</h3>
        <div className="Wrapped-list">
          {procedures.length > 0 ? (
            procedures.map((proc, i) => {
              return (
                <div key={i}>
                  <Procedure name={proc.name} taskList={proc.taskList} />
                  <button
                    type="button"
                    onClick={() => handleEditProcedure(proc)}
                  >
                    Edit Procedure
                  </button>
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

export default ViewProceduresMenu;
