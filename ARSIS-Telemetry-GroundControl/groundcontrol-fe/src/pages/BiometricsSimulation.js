import { useState, useEffect } from 'react';
import UserBiometricsSimulation from '../components/UserBiometricsSimulation';
import './BiometricsSimulation.css';

const BiometricsSimulation = () => {
    const [users, setUsers] = useState([]);
    const [updateIntveral, setUpdateInterval] = useState(1);

    const fetchUsers = async () => {
        const response = await fetch("http://localhost:8080/user");
        const json = await response.json();
        setUsers(json.users);
    };

    useEffect(() => {
        fetchUsers();
    }, []);

    return (
        <>
            <h1>Biometrics Simulation</h1>
            <button onClick={fetchUsers}>Refresh Users</button>
            <br />
            <label>Update Interval: {updateIntveral} update(s) per second</label>
            <br />
            <input type="range" min="0" max="25" value={updateIntveral} onChange={(e) => setUpdateInterval(e.target.value)}/>
            <div className='userPanel'>
                {users.map(user => <UserBiometricsSimulation id={user.id} name={user.name} updateIntveral={updateIntveral}/>)}
            </div>
        </>
    );
}

export default BiometricsSimulation;