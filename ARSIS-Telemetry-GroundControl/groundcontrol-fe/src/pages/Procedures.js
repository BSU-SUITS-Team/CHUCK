import React, { useState } from "react";
import { UpdateProcedureMenu, ViewProceduresMenu, CreateProcedureMenu, SideNav } from "../components"
import "./Procedures.css";

const Procedures = () => {
  const [tab, setTab] = useState(0);

  return (
    <>
      <SideNav />
      <div className="Page">
        <h1>Procedures</h1>
        <p>Ground Control Panel</p>
        <div className="Button-group">
          <button onClick={() => setTab(0)}>View All</button>
          <button onClick={() => setTab(1)}>Create</button>
          <button onClick={() => setTab(2)}>Update</button>
        </div>
        {tab === 0 ? <ViewProceduresMenu /> : <></>}
        {tab === 1 ? <CreateProcedureMenu /> : <></>}
        {tab === 2 ? <UpdateProcedureMenu /> : <></>}
      </div>
    </>
  );
};

export default Procedures;
