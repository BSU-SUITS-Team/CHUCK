import React, { useEffect, useState } from "react";

const UpdateProcedureMenu = () => {
  const [procedures, setProcedures] = useState[{}]

  useEffect(() => {
    fetch("http://localhost:8181/procedures/", {
      method: "GET",
      mode: "cors",
    })
      .then((res) => res.json())
      .then((data) => setProcedures[data]);
  }, [])

  const handleSubmit = (e) => {};

  return (
    <div className="Container-primary">
      <form onSubmit={handleSubmit}>
        <h3>View Procedure Menu</h3>
      </form>
    </div>
  );
};

export default UpdateProcedureMenu;
