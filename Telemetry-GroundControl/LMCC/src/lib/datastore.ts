import { get, writable } from 'svelte/store';

export const datastore = writable({ connected: false });

const MAX_EVENT_LOG_ENTRIES = 1000;
let eventLogSequence = 0;

export type WebSocketEventLogEntry = {
	id: number;
	receivedAt: string;
	type: string;
	label?: string;
	time?: string | number;
	payload: unknown;
	raw: string;
	parseError?: string;
};

export const websocketEventLog = writable<WebSocketEventLogEntry[]>([]);

export function clearWebSocketEventLog() {
	websocketEventLog.set([]);
}

function appendWebSocketEventLog(entry: Omit<WebSocketEventLogEntry, 'id'>) {
	websocketEventLog.update((events) => {
		const nextEvents = [...events, { ...entry, id: ++eventLogSequence }];
		return nextEvents.slice(-MAX_EVENT_LOG_ENTRIES);
	});
}

/**
 * Creates a Svelte store that connects to a WebSocket and listens for JSON messages.
 * @param {string} url - The WebSocket URL to connect to.
 * @returns A Svelte store with the WebSocket's messages.
 */
export function createWebSocketStore(url: string) {
	console.log('Connecting Websocket...');
	const ws = new WebSocket(url);
	let shouldReconnect = true;

	ws.onopen = () => {
		console.log('WebSocket connection established');
		const storedata = get(datastore);
		datastore.set({ ...storedata, connected: true });
	};

	ws.onerror = (error) => {
		console.error('WebSocket error:', error);
	};

	ws.onmessage = (event) => {
		const raw = typeof event.data === 'string' ? event.data : JSON.stringify(event.data);
		const receivedAt = new Date().toISOString();
		try {
			const data = JSON.parse(raw);
			appendWebSocketEventLog({
				receivedAt,
				type: data.type ?? 'unknown',
				label: data.label,
				time: data.time,
				payload: data,
				raw
			});

			const oldStore = get(datastore);
			let newStore = { ...oldStore };
			if (data.label) {
				// perform upsert
				newStore[data.type] ??= {};
				newStore[data.type] = { ...newStore[data.type], [data.label]: data.data };
			} else {
				// perform append
				newStore[data.type] ??= [];
				newStore[data.type] = [...newStore[data.type], { time: data.time, ...data.data }];
			}
			datastore.set(newStore);
		} catch (error) {
			appendWebSocketEventLog({
				receivedAt,
				type: 'parse_error',
				payload: raw,
				raw,
				parseError: error instanceof Error ? error.message : String(error)
			});
			console.error('Error parsing WebSocket message:', error);
		}
	};

	ws.onclose = () => {
		console.log('WebSocket connection closed');
		const storedata = get(datastore);
		datastore.set({ ...storedata, connected: false });
		if (shouldReconnect) {
			setTimeout(() => {
				createWebSocketStore(url);
			}, 1000);
		}
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
			shouldReconnect = false;
			ws.close();
		}
	};
}
