import React, { useContext, useEffect, useState } from "react";
import { ProcedureContext } from "../../pages/Procedures";

const ViewProceduresMenu = () => {
  const [procedures, setProcedures] = useState([]);
  const { func, tabs } = useContext(ProcedureContext);
  const [procedure, setProcedure] = func;
  const [tab, setTab] = tabs;

  const handleRefreshProcedures = async () => {
    const proceduresList = [];
    const response = await fetch("http://localhost:8181/procedures/");
    const data = await response.json();
    for (const [key, value] of Object.entries(data)) {
      proceduresList.push({
        name: key,
        summary: value.summary,
        taskList: [...value.taskList],
      });
    }
    setProcedures(proceduresList);
  };

  useEffect(() => {
    handleRefreshProcedures();
  }, []);

  const handleEditProcedure = (proc) => {
    setProcedure(proc);
    setTab(2);
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
                  <p>{proc.name}</p>
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
