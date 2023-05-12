import "./App.css";
import { BrowserRouter, Routes, Route } from "react-router-dom";
import { Dashboard, Procedures, BiometricsSimulation, Biometrics } from "./pages";

function App() {
  return (
    <BrowserRouter>
      <Routes>
        <Route path="/" element={<Dashboard />} />
        <Route path="/procedures" element={<Procedures />} />
        <Route path="/biometrics" element={<Biometrics />} />
        <Route path="/simulation/biometrics" element={<BiometricsSimulation />} />
      </Routes>
    </BrowserRouter>
  );
}

export default App;
