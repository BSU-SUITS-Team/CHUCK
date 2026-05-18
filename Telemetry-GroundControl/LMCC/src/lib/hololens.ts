import { apiUrl } from '$lib/api';

export type HololensCommandAction = 'open_window' | 'close_window' | 'open_procedure';

export type HololensCommand = {
	action: HololensCommandAction;
	window?: string;
	procedure?: string;
	target?: string;
	source?: string;
};

export type HololensWindowOption = {
	id: string;
	label: string;
	description: string;
};

export const hololensWindows: HololensWindowOption[] = [
	{
		id: 'biometrics',
		label: 'Biometrics',
		description: 'Astronaut vital signs and suit resources'
	},
	{
		id: 'navigation',
		label: 'Navigation',
		description: 'Rockyard map, pins, and navigation context'
	},
	{
		id: 'procedures',
		label: 'Procedures',
		description: 'Procedure browser and task list'
	},
	{
		id: 'spectrometry',
		label: 'Spectrometry',
		description: 'Spectrometry readings and oxide composition'
	},
	{
		id: 'notifications',
		label: 'Notifications',
		description: 'Notification history and active messages'
	},
	{
		id: 'settings',
		label: 'Settings',
		description: 'Unity connection and EVA configuration'
	},
	{
		id: 'summary_timeline',
		label: 'EVA Summary Timeline',
		description: 'Timeline summary of EVA and procedure events'
	}
];

export async function sendHololensCommand(command: HololensCommand) {
	const response = await fetch(apiUrl('/hololens/commands'), {
		method: 'POST',
		headers: {
			'Content-Type': 'application/json'
		},
		body: JSON.stringify(command)
	});

	if (!response.ok) {
		throw new Error(await response.text());
	}

	return response.json();
}

export const openHololensWindow = (window: string) =>
	sendHololensCommand({ action: 'open_window', window });

export const closeHololensWindow = (window: string) =>
	sendHololensCommand({ action: 'close_window', window });

export const openHololensProcedure = (procedure: string) =>
	sendHololensCommand({ action: 'open_procedure', procedure, window: 'procedures' });
