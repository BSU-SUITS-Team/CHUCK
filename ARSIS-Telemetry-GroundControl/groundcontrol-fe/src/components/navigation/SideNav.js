import React from "react";
import logo from "../assets/logo.png";
import { AiFillDashboard } from "react-icons/ai";
import { FcProcess } from "react-icons/fc";
import { FaUserAstronaut } from "react-icons/fa";
import { GiSettingsKnobs, GiTankTread } from "react-icons/gi";
import {
  BsGem,
  BsTextParagraph,
  BsFillGearFill,
  BsFillQuestionCircleFill,
} from "react-icons/bs";

const SideNav = () => {

  return (
    <div className="Side-Nav">
      <img src={logo} alt="app logo" className="App-logo" />
      <div className="Nav-menu">
        <a href="/" className="Nav-link">
          <AiFillDashboard size={"20px"} />
          <p>Dashboard</p>
        </a>
        <a href="/procedures" className="Nav-link">
          <FcProcess size={"20px"} />
          <p>Procedures</p>
        </a>
        <a href="/astronauts" className="Nav-link">
          <FaUserAstronaut size={"20px"} />
          <p>Astronauts</p>
        </a>
        <a href="/rover" className="Nav-link">
          <GiTankTread size={"20px"} />
          <p>Rover</p>
        </a>
        <a href="/uia" className="Nav-link">
          <GiSettingsKnobs size={"20px"} />
          <p>UIA</p>
        </a>
        <a href="/spectrometer" className="Nav-link">
          <BsGem size={"20px"} />
          <p>Spectrometer</p>
        </a>
        <a href="/logs" className="Nav-link">
          <BsTextParagraph size={"20px"} />
          <p>Logs</p>
        </a>
        <a href="/settings" className="Nav-link">
          <BsFillGearFill size={"20px"} />
          <p>Settings</p>
        </a>
        <a href="/help" className="Nav-link">
          <BsFillQuestionCircleFill size={"20px"} />
          <p>Help</p>
        </a>
      </div>
    </div>
  );
};

export default SideNav;
