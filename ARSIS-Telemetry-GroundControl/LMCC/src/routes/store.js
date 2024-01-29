import { writable } from 'svelte/store';

// This has a dataformat of:
// {
//     roverName: [
//         [elementKey, element],
//         [elementKey, element],
//         ...
//     ],
//     roverName: [
//         [elementKey, element],
//         [elementKey, element],
//         ...
//     ],
//     ...
// }
const initialKeepables = {};
// This has a dataformat of:
// {
//     roverName: {
//         graphName: {
//             data: [y1, y2, y3, ...],
//             },
//         },
//         graphName: {
//             data: [y1, y2, y3, ...],
//             }
//         },
//     },
//     roverName: {
//         graphName: {
//             data: [y1, y2, y3, ...],
//             }
//         },
//         graphName: {
//             data: [y1, y2, y3, ...],
//             }
//         },
//     },
//     ...
// }
const initialGraphs = {};

// This has a dataformat of:
// [{notification_name: 'warning'}, {notification_name: 'error'}, ...]
const initialNotifications = [];

function createKeepablesStore() {
    const { subscribe, set, update } = writable(initialKeepables);

    return {
        subscribe,
        removeElement: (roverName, elementKey) => update(n => {
            if (n[roverName]) {
                n[roverName] = n[roverName].filter(([key]) => key !== elementKey);
            }
            // if empty, remove rover
            if (n[roverName].length === 0) {
                delete n[roverName];
            }
            return { ...n };
        }),
        addElement: (roverName, element) => update(n => {
            if (!n[roverName]) {
                n[roverName] = [];
            }
            // if already exists, remove it
            n[roverName] = n[roverName].filter(([key]) => key !== element[0]);
            if (n[roverName]) {
                n[roverName].push(element);
            }
            return { ...n };
        }),
    };
}

function createGraphsStore() {
    const { subscribe, set, update } = writable(initialGraphs);

    return {
        subscribe,
        addGraph: (roverName, graphName, data) => update(n => {
            console.log('addGraph', roverName, graphName, data);
            if (!n[roverName]) {
                n[roverName] = {};
            }
            n[roverName][graphName] = data;
            return { ...n };
        }),
        removeGraph: (roverName, graphName) => update(n => {
            if (n[roverName]) {
                delete n[roverName][graphName];
            }
            // if empty, remove rover
            if (Object.keys(n[roverName]).length === 0) {
                delete n[roverName];
            }
            return { ...n };
        }),
    };
}

function createNotificationsStore() {
    const { subscribe, set, update } = writable(initialNotifications);

    return {
        subscribe,
        addNotification: (name, status) => update(n => {
            console.log('addNotification', name, status);
            n.push({name, status});
            return [...n];
        }),
        removeNotification: (name) => update(n => {
            console.log('removeNotification', name);
            return n.filter(({name: nname}) => nname !== name);
        }),
    };
}


export const keepables = createKeepablesStore();
export const graphdata = createGraphsStore();
export const notifications = createNotificationsStore();