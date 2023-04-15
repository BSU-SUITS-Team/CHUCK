import { useState } from 'react';

const UserBiometrics = ({ id, name }) => {
    const [o2, setO2] = useState(0);
    const [heartrate, setHeartrate] = useState(0);
    const [battery, setBattery] = useState(0);

    const fetchBiometrics = async () => {
        const response = await fetch(`http://localhost:8080/biometrics/${id}`);
        const { o2, heartrate, battery } = await response.json();
        setO2(o2);
        setHeartrate(heartrate);
        setBattery(battery);
    };

    return (
        <div className='user'>
            <h2>ID: {id}, Name: {name}</h2>
            <button onClick={() => fetchBiometrics()}>Update User</button>
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
        </div>
    )
}

export default UserBiometrics;