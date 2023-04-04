import React from "react";
import TaskRow from "../components/TaskRow";
import SideNav from "../components/SideNav";

const Dashboard = () => {
  return (
    <>
      <SideNav />
      <div className="Page">
        <div className="Container-primary">
          <TaskRow />
          <TaskRow />
        </div>
      </div>
    </>
  );
};

export default Dashboard;
