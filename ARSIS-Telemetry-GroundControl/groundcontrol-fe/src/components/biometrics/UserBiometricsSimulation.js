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

const UserBiometrics = ({ id, name, updateInterval }) => {
    const [o2, setO2] = useState(0);
    const [heartrate, setHeartrate] = useState(0);
    const [battery, setBattery] = useState(0);
    const [updateO2, setUpdateO2] = useState(o2);
    const [updateHeartrate, setUpdateHeartrate] = useState(heartrate);
    const [updateBattery, setUpdateBattery] = useState(battery);

    const fetchBiometrics = async () => {
        const response = await fetch(`http://localhost:8080/biometrics/${id}`);
        const { o2, heartrate, battery } = await response.json();
        setO2(o2);
        setHeartrate(heartrate);
        setBattery(battery);
    };

    const putUpdates = async () => {
        const updates = {
            heartrate: parseInt(updateHeartrate),
            o2: parseInt(updateO2),
            battery: parseInt(updateBattery)
        }

        const options = {
            method: "POST",
            mode: "cors",
            headers: {
                "accept": "application/json",
                "Content-Type": "application/json",
            },
            body: JSON.stringify(updates),
        }

        console.log(options.body);

        try {
            const values = await fetch(`http://localhost:8080/biometrics/${id}/update_biometrics`, options)
                .then(response => {
                    if (!response.ok) {
                        return {
                            'battery': 'N/A',
                            'o2': 'N/A',
                            'heartrate': 'N/A'
                        }
                    } else {
                        return response.json();
                    }
                });
            setO2(values['o2']);
            setHeartrate(values['heartrate']);
            setBattery(values['battery']);
        } catch (e) {
            setBattery('N/A');
            setO2('N/A');
            setHeartrate('N/A');
        }
    };

    useEffect(() => {
        fetchBiometrics();
    }, []);

    // useInterval(() => {
    //     putUpdates();
    // }, updateInterval * 10000);

    return (
        <div className='user'>
            <button onClick={() => putUpdates()}>Put Updates</button>
            <h2>ID: {id}, Name: {name}</h2>
            <table>
                <tbody>

                <th colSpan='3'>Current Values</th>
                <tr>
                    <th>Oxygen</th>
                    <th>Heartrate</th>
                    <th>Battery</th>
                </tr>
                <tr>
                    <td>{o2 ?? 'N/A'}</td>
                    <td>{heartrate ?? 'N/A'}</td>
                    <td>{battery ?? 'N/A'}</td>
                </tr>
                </tbody>
            </table>
            <table>
                <tbody>
                <th colSpan='3'>Update Values</th>
                <tr>
                    <th>Oxygen</th>
                    <th>Heartrate</th>
                    <th>Battery</th>
                </tr>
                <tr>
                    <td>{updateO2}<br/><input type="range" min="0" max="100" value={updateO2} onChange={(e) => setUpdateO2(e.target.value)} /></td>
                    <td>{updateHeartrate}<br /><input type="range" min="0" max="250" value={updateHeartrate} onChange={(e) => setUpdateHeartrate(e.target.value)} /></td>
                    <td>{updateBattery}<br /><input type="range" min="0" max="100" value={updateBattery} onChange={(e) => setUpdateBattery(e.target.value)} /></td>
                </tr>
                </tbody>
            </table>
        </div>
    )
};

export default UserBiometrics;