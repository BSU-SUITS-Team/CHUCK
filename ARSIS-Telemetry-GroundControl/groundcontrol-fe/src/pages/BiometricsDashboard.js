import { useState, useEffect } from 'react';
import UserBiometrics from '../components/UserBiometrics';

const BiometricsDashboard = () => {
    const [users, setUsers] = useState([]);

    useEffect(() => {

    }, []);

    return (
        <>
            <h1>Biometrics Dashboard</h1>
            <ul>
                {users.map(user => <UserBiometrics userId={user} />)}
            </ul>
        </>
    );
}

export default BiometricsDashboard;