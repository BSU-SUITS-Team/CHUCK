import React, { useState } from "react";

const Step = (props) => {
  const { step, i, j } = props;
  const [stepType, setStepType] = useState(step.type);
  const [preview, setPreview] = useState(step.body);

  function previewFile(e) {
    const file = e.target.files[0];
    const reader = new FileReader();

    const possibleHeaders = [
      "data:image/png;base64,",
      "data:image/jpg;base64,",
      "data:image/jpeg;base64,",
    ]

    reader.addEventListener(
      "load",
      () => {
        possibleHeaders.forEach(header => {
          if (reader.result.startsWith(header)) {
            setPreview(reader.result.replace(header, ""));
            return;
          }
        });
        alert("File type not supported.")
      },
      false
    );

    if (file) {
      reader.readAsDataURL(file);
    }
  }

  return (
    <div className="Step">
      <label>Step {j + 1}</label>
      <select
        name={"step-type" + i + j}
        defaultValue={step.type}
        onChange={(e) => setStepType(e.target.value)}
      >
        <option value="text">text</option>
        <option value="image">image</option>
      </select>
      {stepType === "text" ? (
        <textarea
          name={"step-body" + i + j}
          defaultValue={step.body}
          placeholder="Body"
          row="5"
          cols="20"
        />
      ) : (
        <div className="Image-upload">
          <input type="file" onChange={(e) => previewFile(e)} />
          <img
            name={"step-body" + i + j}
            src={"data:*/*;base64," + preview}
            alt="uploaded image preview"
          />
        </div>
      )}
    </div>
  );
};

export default Step;
