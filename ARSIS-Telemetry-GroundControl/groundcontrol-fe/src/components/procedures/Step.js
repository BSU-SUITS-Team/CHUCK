import React from "react";

const Step = (props) => {
  const { step, i, j } = props
  return (
    <div className="Step">
      <label>Step {j + 1}</label>
      <select name={"step-type" + i + j}>
        <option value="text" selected={step.type === "text" ? true : false}>
          text
        </option>
        <option value="image" selected={step.type === "image" ? true : false}>
          image
        </option>
      </select>
      <textarea
        name={"step-body" + i + j}
        defaultValue={step.body}
        placeholder="Body"
        row="5"
        cols="20"
      />
    </div>
  );
};

export default Step;
