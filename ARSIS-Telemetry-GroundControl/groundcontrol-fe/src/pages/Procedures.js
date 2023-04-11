import React from "react";
import CreateProcedureMenu from "../components/CreateProcedureMenu"
import SideNav from "../components/SideNav";
import UpdateProcedureMenu from "../components/UpdateProcedureMenu"

const Procedures = () => {
  

  return (
    <>
      <SideNav />
      <div className="Page">
        <h1>Procedures</h1>
        <p>Ground Control Panel</p>
        <CreateProcedureMenu />
        <UpdateProcedureMenu />
      </div>
    </>
  );
};

export default Procedures;
