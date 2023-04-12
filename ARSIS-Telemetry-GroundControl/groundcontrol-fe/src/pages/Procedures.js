import React, { useState } from "react";
import { ViewProceduresMenu, CreateProcedureMenu, UpdateProcedureMenu, SideNav } from "../components"
import "./Procedures.css";

const Procedures = () => {
  const [tab, setTab] = useState(0);

  const [isEditing, setIsEditing] = useState(false)
  const [selectedProc, setSelectedProc] = useState({})

  function handleChangeIsEditing(value) {
    setIsEditing(value)
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
          <button onClick={() => setTab(0)}>View All</button>
          <button onClick={() => setTab(1)}>Create</button>
        </div>
        {tab === 0 && !isEditing ? <ViewProceduresMenu onChangeSelectedProc={handleChangeSelectedProc} onChangeIsEditing={handleChangeIsEditing} /> : <></>}
        {tab === 1 && !isEditing ? <CreateProcedureMenu /> : <></>}
        {isEditing ? <UpdateProcedureMenu selectedProc={selectedProc} onChangeIsEditing={handleChangeIsEditing} /> : <></>}
      </div>
    </>
  );
};

export default Procedures;
