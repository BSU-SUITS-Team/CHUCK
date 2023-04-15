import React, { createContext, useState } from "react";
import { ViewProceduresMenu, CreateProcedureMenu, UpdateProcedureMenu, SideNav } from "../components"
import "./Procedures.css";

export const ProcedureContext = createContext();

const Procedures = () => {
  const defaultProc = {
    name: "",
    summary: "",
    taskList: [
      {
        name: "",
        summary: "",
        stepList: [{ type: "image", body: "", nextTask: "" }],
      },
      {
        name: "",
        summary: "",
        stepList: [{ type: "image", body: "", nextTask: "" }],
      },
      {
        name: "",
        summary: "",
        stepList: [{ type: "image", body: "", nextTask: "" }],
      },
    ],
  };
  const [tab, setTab] = useState(0);
  const [procedure, setProcedure] = useState(defaultProc)

  function handleChangeTab(value = 0) {
    if (value === 1) {
      setProcedure(defaultProc)
    }
    setTab(value)
  }
  
  return (
    <ProcedureContext.Provider value={{ func: [procedure, setProcedure], tabs: [tab, handleChangeTab]}}>
      <SideNav />
      <div className="Page">
        <h1>Procedures</h1>
        <p>Ground Control Panel</p>
        <div className="Button-group">
          <button onClick={() => handleChangeTab(0)}>View All</button>
          <button onClick={() => handleChangeTab(1)}>Create</button>
        </div>
        {tab === 0 ? <ViewProceduresMenu/> : <></>}
        {tab === 1 ? <CreateProcedureMenu /> : <></>}
        {tab === 2 ? <UpdateProcedureMenu /> : <></>}
      </div>
    </ProcedureContext.Provider>
  );
};

export default Procedures;
