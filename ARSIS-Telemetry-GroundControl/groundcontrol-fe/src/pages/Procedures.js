import React, { useState } from "react";
import { ViewProceduresMenu, CreateProcedureMenu, UpdateProcedureMenu, SideNav } from "../components"
import "./Procedures.css";

const Procedures = () => {
  const [tab, setTab] = useState(0);
  const [selectedProc, setSelectedProc] = useState({})

  function handleChangeTab(value = 0) {
    setTab(value)
  }

  function handleChangeSelectedProc(value) {
    setSelectedProc(value)
  }
  
  return (
    <>
      <SideNav />
      <div className="Page">
        <h1>Procedures</h1>
        <p>Ground Control Panel</p>
        <div className="Button-group">
          <button onClick={() => handleChangeTab(0)}>View All</button>
          <button onClick={() => handleChangeTab(1)}>Create</button>
        </div>
        {tab === 0 ? <ViewProceduresMenu onChangeSelectedProc={handleChangeSelectedProc} onChangeTab={handleChangeTab} /> : <></>}
        {tab === 1 ? <CreateProcedureMenu onChangeTab={handleChangeTab} /> : <></>}
        {tab === 2 ? <UpdateProcedureMenu selectedProc={selectedProc} onChangeTab={handleChangeTab}/> : <></>}
      </div>
    </>
  );
};

export default Procedures;
