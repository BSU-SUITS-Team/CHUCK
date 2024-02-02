import { readable, writable } from 'svelte/store';

export const numberofastronauts = writable(1);
export const biometics = readable(
    {
        name: 'EV-1',
        oxygen: 99,
        battery: 14,
        co2: 178,
        heartRate: 90,
        temperature: 98,
        location: 'Mars',
        fanSpeed: 68
    },
    (set) => {
        const interval = setInterval(() => {
            set({
                name: 'EV-1',
                oxygen: Math.floor(Math.random() * 100),
                battery: Math.floor(Math.random() * 100),
                co2: Math.floor(Math.random() * 100),
                heartRate: Math.floor(Math.random() * 100),
                temperature: Math.floor(Math.random() * 100),
                location: 'Mars',
                fanSpeed: Math.floor(Math.random() * 100)
            });
        }, 1000);

        return () => clearInterval(interval);
    }
);
