<script lang="ts">
	import { Button, Badge } from 'flowbite-svelte';
	import {
		clearWebSocketEventLog,
		websocketEventLog,
		type WebSocketEventLogEntry
	} from '$lib/datastore';

	let searchTerm = '';
	let selectedType = 'all';
	let selectedLabel = 'all';
	let expandedEventId: number | undefined;

	const formatReceivedAt = (value: string) =>
		new Intl.DateTimeFormat(undefined, {
			hour: '2-digit',
			minute: '2-digit',
			second: '2-digit',
			fractionalSecondDigits: 3
		}).format(new Date(value));

	const stringifyPayload = (payload: unknown) => {
		try {
			return JSON.stringify(payload, null, 2);
		} catch {
			return String(payload);
		}
	};

	const getSearchTarget = (event: WebSocketEventLogEntry) =>
		[event.type, event.label, event.time, event.receivedAt, event.parseError, event.raw]
			.filter(Boolean)
			.join(' ')
			.toLowerCase();

	const matchesSearch = (event: WebSocketEventLogEntry) => {
		const normalizedSearch = searchTerm.trim().toLowerCase();
		return normalizedSearch === '' || getSearchTarget(event).includes(normalizedSearch);
	};

	const clearLog = () => {
		clearWebSocketEventLog();
		searchTerm = '';
		selectedType = 'all';
		selectedLabel = 'all';
		expandedEventId = undefined;
	};

	$: typeOptions = Array.from(new Set($websocketEventLog.map((event) => event.type))).sort();
	$: labelOptions = Array.from(
		new Set($websocketEventLog.map((event) => event.label).filter(Boolean) as string[])
	).sort();
	$: filteredEvents = $websocketEventLog
		.filter((event) => selectedType === 'all' || event.type === selectedType)
		.filter((event) => selectedLabel === 'all' || event.label === selectedLabel)
		.filter(matchesSearch)
		.slice()
		.reverse();
</script>

<svelte:head>
	<title>WebSocket Event Log</title>
</svelte:head>

<div class="h-full p-4 text-gray-900 dark:text-gray-100">
	<div class="flex flex-wrap items-center justify-between gap-3 mb-4">
		<div>
			<h2 class="text-2xl font-semibold leading-tight">WebSocket Event Log</h2>
			<div class="mt-2 flex flex-wrap gap-2">
				<Badge color="blue">{$websocketEventLog.length} received</Badge>
				<Badge color="gray">{filteredEvents.length} shown</Badge>
			</div>
		</div>
		<Button color="alternative" onclick={clearLog}>Clear</Button>
	</div>

	<div
		class="grid gap-3 rounded border border-gray-200 bg-white p-3 dark:border-gray-700 dark:bg-gray-800 md:grid-cols-[minmax(16rem,1fr)_12rem_12rem]"
	>
		<label class="block text-sm font-medium text-gray-700 dark:text-gray-300">
			Search
			<input
				class="mt-1 block w-full rounded border border-gray-300 bg-gray-50 p-2 text-sm text-gray-900 focus:border-blue-500 focus:ring-blue-500 dark:border-gray-600 dark:bg-gray-700 dark:text-white"
				type="search"
				bind:value={searchTerm}
				placeholder="type, label, payload"
			/>
		</label>

		<label class="block text-sm font-medium text-gray-700 dark:text-gray-300">
			Type
			<select
				class="mt-1 block w-full rounded border border-gray-300 bg-gray-50 p-2 text-sm text-gray-900 focus:border-blue-500 focus:ring-blue-500 dark:border-gray-600 dark:bg-gray-700 dark:text-white"
				bind:value={selectedType}
			>
				<option value="all">All types</option>
				{#each typeOptions as type}
					<option value={type}>{type}</option>
				{/each}
			</select>
		</label>

		<label class="block text-sm font-medium text-gray-700 dark:text-gray-300">
			Label
			<select
				class="mt-1 block w-full rounded border border-gray-300 bg-gray-50 p-2 text-sm text-gray-900 focus:border-blue-500 focus:ring-blue-500 dark:border-gray-600 dark:bg-gray-700 dark:text-white"
				bind:value={selectedLabel}
			>
				<option value="all">All labels</option>
				{#each labelOptions as label}
					<option value={label}>{label}</option>
				{/each}
			</select>
		</label>
	</div>

	<div class="mt-4 overflow-x-auto rounded border border-gray-200 dark:border-gray-700">
		<div class="min-w-[48rem]">
			<div
				class="grid grid-cols-[9rem_9rem_11rem_minmax(16rem,1fr)] gap-3 border-b border-gray-200 bg-gray-50 px-4 py-2 text-xs font-semibold uppercase text-gray-500 dark:border-gray-700 dark:bg-gray-800 dark:text-gray-400"
			>
				<span>Received</span>
				<span>Type</span>
				<span>Label</span>
				<span>Payload</span>
			</div>

			<div class="max-h-[calc(100vh-19rem)] overflow-y-auto bg-white dark:bg-gray-900">
				{#if filteredEvents.length === 0}
					<div class="px-4 py-8 text-center text-sm text-gray-500 dark:text-gray-400">
						No events match the current filters.
					</div>
				{:else}
					{#each filteredEvents as event (event.id)}
						<button
							type="button"
							class="grid w-full grid-cols-[9rem_9rem_11rem_minmax(16rem,1fr)] gap-3 border-b border-gray-100 px-4 py-3 text-left text-sm hover:bg-gray-50 dark:border-gray-800 dark:hover:bg-gray-800"
							onclick={() =>
								(expandedEventId = expandedEventId === event.id ? undefined : event.id)}
						>
							<span class="font-mono text-xs text-gray-600 dark:text-gray-300">
								{formatReceivedAt(event.receivedAt)}
							</span>
							<span>
								<Badge color={event.parseError ? 'red' : 'gray'}>{event.type}</Badge>
							</span>
							<span class="truncate text-gray-700 dark:text-gray-300">
								{event.label ?? '-'}
							</span>
							<span class="truncate font-mono text-xs text-gray-600 dark:text-gray-400">
								{event.raw}
							</span>
						</button>

						{#if expandedEventId === event.id}
							<div
								class="border-b border-gray-100 bg-gray-50 px-4 py-3 dark:border-gray-800 dark:bg-gray-950"
							>
								{#if event.parseError}
									<p class="mb-2 text-sm font-medium text-red-600 dark:text-red-400">
										{event.parseError}
									</p>
								{/if}
								<pre
									class="max-h-96 overflow-auto rounded bg-gray-900 p-3 text-xs text-gray-100">{stringifyPayload(
										event.payload
									)}</pre>
							</div>
						{/if}
					{/each}
				{/if}
			</div>
		</div>
	</div>
</div>
