import "./App.css";
import { BrowserRouter, Routes, Route } from "react-router-dom";
import { Dashboard, Procedures } from "./pages";

function App() {
  return (
    <BrowserRouter>
      <Routes>
        <Route path="/" element={<Dashboard />} />
        <Route path="/procedures" element={<Procedures />} />
      </Routes>
    </BrowserRouter>
  );
}

export default App;
