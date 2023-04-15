import { useState, useEffect } from 'react';
import SideNav from '../components/SideNav';
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
            <SideNav />
            <div className='Page'>
                <h1>Biometrics Simulation</h1>
                <button onClick={fetchUsers}>Fetch Users</button>
                {/* <br />
                <label>Update Interval: {updateIntveral} update(s) per second</label>
                <br /> */}
                <input type="range" min="0" max="25" value={updateIntveral} onChange={(e) => setUpdateInterval(e.target.value)}/>
                <div className='userPanel'>
                    {users.map(user => <UserBiometricsSimulation key={user.id} id={user.id} name={user.name}/>)}
                </div>
            </div>
        </>
    );
}

export default BiometricsSimulation;