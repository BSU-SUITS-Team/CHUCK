import React, { useContext, useEffect, useState } from "react";
import { ProcedureContext } from "../../pages/Procedures";
import { w3cwebsocket as W3CWebSocket } from "websocket"

const ViewProceduresMenu = () => {
  const [procedures, setProcedures] = useState([]);
  const { func, tabs } = useContext(ProcedureContext);
  const [procedure, setProcedure] = func;
  const [tab, setTab] = tabs;

  const client = new W3CWebSocket('ws://localhost:8181/ws/updates');

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

  const handleSyncProcedures = async () => {
    const message = '{"name": "PROCEDURES"}'
    client.send(message)
  }

  useEffect(() => {
    client.onopen = () => {
        console.log('WebSocket Client Connected');
    };
  
    client.onmessage = (message) => {
        console.log(message);
    };
    client.onerror = function() {
        console.log('Connection Error');
    };
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
                <div key={i} className="Procedure">
                  <h2>{proc.name}</h2>
                  {proc.taskList.map((task, j) => {
                    return (<p key={j}>{task.name}</p>)
                  })}
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
        <button type="button" onClick={handleSyncProcedures}>
          Sync
        </button>
      </form>
    </div>
  );
};

export default ViewProceduresMenu;
