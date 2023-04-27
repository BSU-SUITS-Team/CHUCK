import { useState, useEffect } from 'react';
import SideNav from '../components/navigation/SideNav';
import UserBiometrics from '../components/biometrics/UserBiometrics';
import './BiometricsSimulation.css';

const Biometrics = () => {
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
                <button onClick={() => fetchUsers()}>Fetch Users</button>
                <div className='userPanel'>
                    {users.map(user => <UserBiometrics key={user.id} id={user.id} name={user.name} />)}
                </div>
            </div>
        </>
    );
}

export default Biometrics;