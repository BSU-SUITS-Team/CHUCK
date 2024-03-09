import { get, writable } from 'svelte/store';

/**
 * Creates a Svelte store that connects to a WebSocket and listens for JSON messages.
 * @param {string} url - The WebSocket URL to connect to.
 * @returns A Svelte store with the WebSocket's messages.
 */
export function createWebSocketStore(url: string) {
	const store = writable(null);

	const ws = new WebSocket(url);

	ws.onopen = () => {
		console.log('WebSocket connection established');
	};

	ws.onerror = (error) => {
		console.error('WebSocket error:', error);
	};

	ws.onmessage = (event) => {
		try {
			const data = JSON.parse(event.data);
			console.log(data);
			let oldStore = get(store)
			let upsert = oldStore ? oldStore.upsert : null;
			let lastlist = oldStore ? oldStore[data.type] : [];
			lastlist = lastlist ? lastlist : [];
			store.set({ ...oldStore, ...{ [data.type]: {...lastlist, ...{ [(upsert ? upsert : data.time)]: Object.values(data.data) }}}})

		} catch (error) {
			console.error('Error parsing WebSocket message:', error);
		}
	};

	ws.onclose = () => {
		console.log('WebSocket connection closed');
	};

	return {
		subscribe: store.subscribe,
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
