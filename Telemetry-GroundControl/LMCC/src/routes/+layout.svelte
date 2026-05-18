<script>
	import '../app.postcss';
	import Sidebar from './Sidebar.svelte';
	import { slide } from 'svelte/transition';
	import { graphdata, keepables, notifications } from './store';
	import { Heading, Button, Span, Toast, Badge } from 'flowbite-svelte';
	import {
		TrashBinOutline,
		EditSolid,
		ExclamationCircleOutline,
		AnnotationOutline,
		LightbulbOutline
	} from 'flowbite-svelte-icons';
	import TinyGraph from './TinyGraph.svelte';
	import { getWebSocketEventsUrl } from '$lib/api';
	import { createWebSocketStore, datastore } from '$lib/datastore';
	import { onDestroy } from 'svelte';
	import { browser } from '$app/environment';
	import { formatTime } from '$lib/formatting';

	if (browser) {
		document.documentElement.classList.remove('dark');
		localStorage.setItem('THEME_PREFERENCE_KEY', 'light');
		localStorage.setItem('color-theme', 'light');
		localStorage.setItem('theme', 'light');

		const websocket = createWebSocketStore(getWebSocketEventsUrl());
		const unsubscribe = datastore.subscribe(() => {});
		onDestroy(() => {
			unsubscribe();
			websocket.close();
		});
	}

	//notification handling
	const SEVERITIES = { 0: 'error', 1: 'warn', 2: 'info' };
	datastore.subscribe((store) => {
		if (store['notification']) {
			for (let i = 0; i < store['notification'].length; i++) {
				let n = store['notification'][i];
				notifications.addNotification(n.content, SEVERITIES[n.severity], n.time);
			}
		}
	});

	//datastore.subscribe(console.log);

	const getElapsedTime = (store) => {
		const telemetry = store.telemetry?.[store.telemetry.length - 1];
		if (!telemetry) return undefined;

		const elapsedTimes = Object.entries(telemetry)
			.filter(([key, value]) => key !== 'time' && value && typeof value === 'object')
			.map(([, eva]) => eva.eva_elapsed_time)
			.filter((value) => Number.isFinite(value));

		if (!elapsedTimes.length) return undefined;
		return Math.max(...elapsedTimes);
	};

	$: hasSideBar = Object.keys($keepables).length > 0 || Object.keys($graphdata).length > 0;
	$: elapsedTime = getElapsedTime($datastore);
</script>

<div class="flex flex-col h-screen bg-slate-50 text-slate-900">
	<div
		class="h-16 border-b border-slate-200 bg-white p-4 flex flex-row justify-between text-slate-900 text-lg"
	>
		<div class="flex flex-row">
			<p class="pr-12">Oxygen: <span class="text-blue-600 font-bold">96 Minuties</span></p>
			{#if elapsedTime !== undefined}
				<p>Elapsed Time: {formatTime(elapsedTime)}</p>
			{/if}
		</div>
		<div>
			{#if $datastore.connected}
				<Badge color="green" class="ml-4">Connected</Badge>
			{:else}
				<Badge class="ml-4">Connecting</Badge>
			{/if}
		</div>
	</div>
	<div class="bg-slate-50 flex overflow-hidden h-full">
		<aside
			class="absolute flex-grow-0 flex-shrink-0 w-fit
					flex-col justify-between flex bg-white border-r border-slate-200"
			style="height: calc(100vh - 4rem);"
		>
			<span class="p-8 text-center">
				<Heading tag="h1">CHUCK</Heading>
			</span>
			<div class="min-h-0 flex-1 overflow-y-auto">
				<Sidebar />
			</div>
		</aside>

		<main
			class="flex flex-col flex-1 ml-64 overflow-y-auto {hasSideBar ? 'mr-72' : ''} hide-scrollbar"
		>
			<div class="absolute right-5 top-16 pt-1 z-50">
				{#each $notifications as notification}
					{#if notification['status'] == 'error'}
						<Toast transition={slide} class="mb-2" color="red">
							<ExclamationCircleOutline slot="icon" class="w-5 h-5" />
							{notification['name']}
						</Toast>
					{:else if notification['status'] == 'warn'}
						<Toast transition={slide} class="mb-2">
							<AnnotationOutline slot="icon" class="w-5 h-5" />
							{notification['name']}
						</Toast>
					{:else if notification['status'] == 'info'}
						<Toast transition={slide} class="mb-2" color="gray">
							<LightbulbOutline slot="icon" class="w-5 h-5" />
							{notification['name']}
						</Toast>
					{/if}
				{/each}
			</div>
			<slot />
		</main>

		{#if hasSideBar}
			<div
				class="absolute right-0 p-5 bg-white overflow-y-auto h-full border-l border-slate-200"
				style="width: 18rem; height: calc(100vh - 4rem);"
			>
				{#each new Set([...Object.keys($keepables), ...Object.keys($graphdata)]) as label}
					<div class="border-b border-slate-200 flex-row flex pb-2 mb-1">
						<Heading tag="h4">{label}</Heading>
						<EditSolid class="mr-2 h-7 text-gray-800" href="/rover" />
						<TrashBinOutline class="h-7 text-gray-800" />
					</div>

					{#if $keepables[label]}
						<div class="flex justify-left pr-2 pl-2 flex-wrap">
							{#each $keepables[label] as item}
								<div class="p-1">
									<Button
										color="alternative"
										onclick={() => keepables.removeElement(label, item[0])}
									>
										{item[0]}&nbsp
										<Span highlight>{item[1]}</Span>
									</Button>
								</div>
							{/each}
						</div>
					{/if}
					{#if $graphdata[label]}
						{#each Object.keys($graphdata[label]) as graph}
							<div
								class="h-52 flex p-3"
								onclick={() => graphdata.removeGraph(label, graph)}
								role="button"
								tabindex="0"
								onkeydown={() => graphdata.removeGraph(label, graph)}
							>
								<TinyGraph
									graphdata={$graphdata[label][graph]}
									name={graph}
									status={$graphdata[label][graph][$graphdata[label][graph].length - 1]}
								/>
							</div>
						{/each}
					{/if}
				{/each}
			</div>
		{/if}
	</div>
</div>

<style>
	/* Utilities for hiding scrollbars */

	/* For Chrome, Safari, and newer versions of Opera */
	.hide-scrollbar::-webkit-scrollbar {
		width: 0; /* For vertical scrollbars */
		height: 0; /* For horizontal scrollbars */
	}

	/* For Firefox */
	.hide-scrollbar {
		scrollbar-width: none;
	}

	/* For Internet Explorer and Edge */
	.hide-scrollbar {
		-ms-overflow-style: none;
	}
</style>
