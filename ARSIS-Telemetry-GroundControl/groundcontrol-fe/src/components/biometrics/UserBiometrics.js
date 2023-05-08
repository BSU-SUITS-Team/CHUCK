import React, { useEffect, useState } from 'react';
import Highcharts from 'highcharts'
import HighchartsReact from 'highcharts-react-official'

const UserBiometrics = ({ id, name }) => {
    const [vent, setVent] = useState(true);
    const [sop, setSOP] = useState(true);
    const [options, setOptions] = useState([
        {
            chart: {
                width: 500
            },
            xAxis: {
                type: 'datetime'
            },
            title: {
                text: `Biometrics for ID: ${id}, Name: ${name}`
            },
            series: [
                { type: 'spline', name: 'o2', data: [] },
                { type: 'spline', name: 'heartrate', data: [] },
                { type: 'spline', name: 'battery', data: [] },
                { type: 'spline', name: 'co2', data: [] },
                { type: 'spline', name: 'suitPressure', data: [] },
            ],
        },
        {
            chart: {
                width: 500
            },
            xAxis: {
                type: 'datetime'
            },
            title: {
                text: `Fan Speed`
            },
            series: [
                { type: 'spline', name: 'fan', data: [] }
            ]
        }
    ])

    const [fanOptions, setFanOptions] = useState({
        
    })

    const requestData = async () => {
        try {
            const request = await fetch(`http://localhost:8080/biometrics/${id}/`);
            const response = await request.json();
            const timestamp = Date.now()

            setVent(response['vent'])
            setSOP(response['sop'])

            let updatedSeries = options[0].series;
            updatedSeries.forEach((value, index) => {
                updatedSeries[index].data.push([timestamp, response[value.name]])

                if (value.data.length > 25) {
                    updatedSeries[index].data = updatedSeries[index].data.slice(1);
                }
            });

            let fanSeries = options[1].series;
            fanSeries[0].data.push([timestamp, response['fan']]);

            if (fanSeries[0].data.length > 25) {
                fanSeries[0].data = fanSeries[0].data.slice(1);
            }

            setOptions([
                {
                    xAxis: {
                        type: 'datetime'
                    },
                    title: {
                        text: `Biometrics for ID: ${id}, Name: ${name}`
                    },
                    series: updatedSeries
                },
                {
                    xAxis: {
                        type: 'datetime'
                    },
                    title: {
                        text: `Fan Speed`
                    },
                    series: fanSeries
                }
            ]);
        } catch (e) {
            console.log(e)
        } finally {
            setTimeout(requestData, 1000)
        }
    }

    useEffect(() => { requestData() }, []);

    useEffect(() => { console.log(options) }, [options]);

    return (
        <div className='user'>
            {/* <h2>ID: {id}, Name: {name}</h2> */}
            <div>
                <p>Vent: {vent ? '✅' : '❌'}</p>
                <p>Secondary O2: {sop ? '✅' : '❌'}</p>
            </div>
            {options.map(i => 
                <HighchartsReact
                    highcharts={Highcharts}
                    options={i}
                />
            )}
            {/* <table>
                <tbody>
                    <tr>
                        <th colSpan='3'>Current Values</th>
                    </tr>
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
                    <tr>
                        <th colSpan='3'>Update Values</th>
                    </tr>
                    <tr>
                        <th>Oxygen</th>
                        <th>Heartrate</th>
                        <th>Battery</th>
                    </tr>
                    <tr>
                        <td>{updateO2}<br /><input type="range" min="0" max="100" value={updateO2} onChange={(e) => setUpdateO2(e.target.value)} /></td>
                        <td>{updateHeartrate}<br /><input type="range" min="0" max="250" value={updateHeartrate} onChange={(e) => setUpdateHeartrate(e.target.value)} /></td>
                        <td>{updateBattery}<br /><input type="range" min="0" max="100" value={updateBattery} onChange={(e) => setUpdateBattery(e.target.value)} /></td>
                    </tr>
                </tbody>
            </table> */}
        </div>
    )
}

export default UserBiometrics