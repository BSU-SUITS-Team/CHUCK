import { useState, useEffect, useRef } from 'react';

const useInterval = (callback, delay) => {
    const savedCallback = useRef();

    useEffect(() => {
        savedCallback.current = callback;
    }, [callback]);

    useEffect(() => {
        const tick = () => {
            savedCallback.current();
        }
        if (delay !== null) {
            let id = setInterval(tick, delay);
            return () => clearInterval(id);
        }
    }, [delay]);
};

const UserBiometricsSimulation = ({ id, name }) => {
    const [o2, setO2] = useState(0);
    const [heartrate, setHeartrate] = useState(0);
    const [battery, setBattery] = useState(0);
    const [fan, setFan] = useState(0);
    const [vent, setVent] = useState(true);
    const [co2, setCo2] = useState(0);
    const [sop, setSop] = useState(true);
    const [suitPressure, setSuitPressure] = useState(0);
    const [updateO2, setUpdateO2] = useState(o2);
    const [updateHeartrate, setUpdateHeartrate] = useState(heartrate);
    const [updateBattery, setUpdateBattery] = useState(battery);
    const [updateFan, setUpdateFan] = useState(fan);
    const [updateVent, setUpdateVent] = useState(vent);
    const [updateCO2, setUpdateCO2] = useState(co2);
    const [updateSOP, setUpdateSOP] = useState(sop);
    const [updateSuitPressure, setUpdateSuitPressure] = useState(suitPressure);

    const fetchBiometrics = async () => {
        const response = await fetch(`http://localhost:8080/biometrics/${id}/`);
        const { o2, heartrate, battery, fan, vent, co2, sop, suitPressure } = await response.json();
        setO2(o2);
        setHeartrate(heartrate);
        setBattery(battery);
        setFan(fan);
        setVent(vent);
        setCo2(co2);
        setSop(sop);
        setSuitPressure(suitPressure);
    };

    const putUpdates = async () => {
        const updates = {
            heartrate: parseInt(updateHeartrate),
            o2: parseInt(updateO2),
            battery: parseInt(updateBattery),
            fan: parseInt(updateFan),
            vent: updateVent,
            co2: parseInt(updateCO2),
            sop: updateSOP,
            suitPressure: parseInt(updateSuitPressure)
        }

        const options = {
            method: "POST",
            mode: "cors",
            headers: {
                "Accept": "application/json",
                "Content-Type": "application/json",
            },
            body: JSON.stringify(updates),
        }

        try {
            const values = await fetch(`http://localhost:8080/biometrics/${id}/update_biometrics/`, options)
                .then(response => {
                    if (!response.ok) {
                        return {
                            'battery': 'N/A',
                            'o2': 'N/A',
                            'heartrate': 'N/A',
                            'fan': 'N/A',
                            'vent': 'N/A',
                            'co2': 'N/A',
                            'sop': 'N/A',
                            'suitPressure': 'N/A'
                            
                        }
                    } else {
                        return response.json();
                    }
                });
            setO2(values['o2']);
            setHeartrate(values['heartrate']);
            setBattery(values['battery']);
            setFan(values['fan']);
            setVent(values['vent']);
            setCo2(values['co2']);
            setSop(values['sop']);
            setSuitPressure(values['suitPressure']);
        } catch (e) {
            setBattery('N/A');
            setO2('N/A');
            setHeartrate('N/A');
            setFan('N/A');
            setVent('N/A');
            setCo2('N/A');
            setSop('N/A');
            setSuitPressure('N/A');
        }
    };

    useEffect(() => {
        fetchBiometrics();
    }, []);

    useInterval(async () => {
        await putUpdates();
    }, 1000);

    return (
        <div className='user'>
            <button onClick={() => putUpdates()}>Put Updates</button>
            <h2>ID: {id}, Name: {name}</h2>
            <table>
                <tbody>
                    <tr>
                        <th colSpan='3'>Current Values</th>
                    </tr>
                    <tr>
                        <th>Oxygen</th>
                        <th>Heartrate</th>
                        <th>Battery</th>
                        <th>Fan</th>
                        <th>Vent</th>
                        <th>CO2</th>
                        <th>Secondary O2</th>
                        <th>Suit Pressure</th>
                    </tr>
                    <tr>
                        <td>{o2 ?? 'N/A'}</td>
                        <td>{heartrate ?? 'N/A'}</td>
                        <td>{battery ?? 'N/A'}</td>
                        <td>{fan ?? 'N/A'}</td>
                        <td>{vent ? '✅' : '❌' ?? 'N/A'}</td>
                        <td>{co2 ?? 'N/A'}</td>
                        <td>{sop ? '✅' : '❌' ?? 'N/A'}</td>
                        <td>{suitPressure ?? 'N/A'}</td>
                    </tr>
                </tbody>
            </table>
            <table>
                <tbody>
                    <tr>
                        <th colSpan='3'>Update Values</th>
                    </tr>
                    <tr>
                        <th>Oxygen</th>
                        <th>Heartrate</th>
                        <th>Battery</th>
                        <th>Fan</th>
                        <th>Vent</th>
                        <th>CO2</th>
                        <th>Secondary O2</th>
                        <th>Suit Pressure</th>
                    </tr>
                    <tr>
                        <td>{updateO2}<br/><input type="range" min="0" max="100" value={updateO2} onChange={(e) => setUpdateO2(e.target.value)} /></td>
                        <td>{updateHeartrate}<br /><input type="range" min="0" max="250" value={updateHeartrate} onChange={(e) => setUpdateHeartrate(e.target.value)} /></td>
                        <td>{updateBattery}<br /><input type="range" min="0" max="100" value={updateBattery} onChange={(e) => setUpdateBattery(e.target.value)} /></td>
                        <td>{updateFan}<br /><input type="range" min="0" max="20000" value={updateFan} onChange={(e) => setUpdateFan(e.target.value)} /></td>
                        <td>{updateVent}<br /><input type="checkbox" checked={updateVent} onChange={(e) => setUpdateVent(e.target.checked)} /></td>
                        <td>{updateCO2}<br /><input type="range" min="0" max="1000" value={updateCO2} onChange={(e) => setUpdateCO2(e.target.value)} /></td>
                        <td>{updateSOP}<br /><input type="checkbox" checked={updateSOP} onChange={(e) => setUpdateSOP(e.target.checked)} /></td>
                        <td>{updateSuitPressure}<br /><input type="range" min="0" max="10" value={updateSuitPressure} onChange={(e) => setUpdateSuitPressure(e.target.value)} /></td>
                    </tr>
                </tbody>
            </table>
        </div>
    )
};

export default UserBiometricsSimulation;