import { get, writable } from 'svelte/store';

export const datastore = writable({ connected: false });

/**
 * Creates a Svelte store that connects to a WebSocket and listens for JSON messages.
 * @param {string} url - The WebSocket URL to connect to.
 * @returns A Svelte store with the WebSocket's messages.
 */
export function createWebSocketStore(url: string) {
	console.log('Connecting Websocket...');
	const ws = new WebSocket(url);

	ws.onopen = () => {
		console.log('WebSocket connection established');
		const storedata = get(datastore);
		storedata.connected = true;
		datastore.set(storedata);
	};

	ws.onerror = (error) => {
		console.error('WebSocket error:', error);
	};

	ws.onmessage = (event) => {
		try {
			const data = JSON.parse(event.data);
			const oldStore = get(datastore);
			let newStore = oldStore;
			if (data.label) {
				// perform upsert
				newStore[data.type] ??= {};
				newStore[data.type][data.label] = data.data;
			} else {
				// perform append
				newStore[data.type] ??= [];
				newStore[data.type] = [...newStore[data.type], { time: data.time, ...data.data }];
			}
			datastore.set(newStore);
		} catch (error) {
			console.error('Error parsing WebSocket message:', error);
		}
	};

	ws.onclose = () => {
		console.log('WebSocket connection closed');
		const storedata = get(datastore);
		storedata.connected = false;
		datastore.set(storedata);
		setTimeout(() => {
			createWebSocketStore(url);
		}, 1000);
	};

	return {
		subscribe: datastore.subscribe,
		send: (data: string) => {
			if (ws.readyState === WebSocket.OPEN) {
				ws.send(JSON.stringify(data));
			} else {
				console.error('WebSocket is not open. Message not sent.');
			}
		},
		close: () => {
			ws.close();
		}
	};
}
