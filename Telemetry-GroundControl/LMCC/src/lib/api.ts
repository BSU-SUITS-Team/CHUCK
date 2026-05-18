import { browser } from '$app/environment';

const DEFAULT_API_PORT = '8181';
const DEFAULT_API_ORIGIN = `http://localhost:${DEFAULT_API_PORT}`;

function trimTrailingSlash(value: string) {
	return value.replace(/\/+$/, '');
}

export function getApiOrigin() {
	const configuredOrigin = import.meta.env.VITE_GROUNDCONTROL_API_URL?.trim();
	if (configuredOrigin) {
		return trimTrailingSlash(configuredOrigin);
	}

	if (browser) {
		const port = import.meta.env.VITE_GROUNDCONTROL_API_PORT?.trim() || DEFAULT_API_PORT;
		return `${window.location.protocol}//${window.location.hostname}:${port}`;
	}

	return DEFAULT_API_ORIGIN;
}

export function apiUrl(path: string) {
	return `${getApiOrigin()}${path.startsWith('/') ? path : `/${path}`}`;
}

export function getWebSocketEventsUrl() {
	const configuredUrl = import.meta.env.VITE_GROUNDCONTROL_WS_URL?.trim();
	if (configuredUrl) {
		return configuredUrl;
	}

	return `${getApiOrigin().replace(/^http/i, 'ws')}/ws/events`;
}
