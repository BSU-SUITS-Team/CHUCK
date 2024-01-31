<script>
	import '../app.postcss';
	import Sidebar from './Sidebar.svelte';
	import { slide } from 'svelte/transition';
	import { graphdata, keepables, notifications } from './store.js';
	import { Heading, DarkMode, Button, Span, Toast } from 'flowbite-svelte';
	import {
		TrashBinOutline,
		FileEditSolid,
		ExclamationCircleOutline,
		AnnotationOutline,
		LightbulbOutline
	} from 'flowbite-svelte-icons';
	import TinyGraph from './TinyGraph.svelte';

	let hasNotification = false;
	let notificationText = 'Oxygen Tank Has Exploaded';

	function notify(text) {
		notificationList.push(text);
		notificationText = text;
		hasNotification = true;
		setTimeout(() => {
			notificationList.shift();
			if (notificationList.length > 0) {
				notificationText = notificationList[0];
			} else {
				hasNotification = false;
			}
		}, 15000);
	}

	$: hasSideBar = Object.keys($keepables).length > 0 || Object.keys($graphdata).length > 0;
</script>

<div class="flex flex-col h-screen">
	<div
		class="h-16 border-b p-4 flex flex-row justify-between dark:bg-gray-800 dark:border-gray-700 dark:text-gray-300 text-lg"
	>
		<div class="flex flex-row">
			<p class="pr-12">Oxygen: <span class="text-blue-600 font-bold">96 Minuties<span /></span></p>
			<p>32 Minuties Elapsed</p>
		</div>
		<p>Other Important Text That Is Longer and Sort of Just Sits at the Top Providing Status</p>
	</div>
	<div class="dark:bg-gray-900 flex overflow-hidden h-full">
		<aside
			class="absolute flex-grow-0 flex-shrink-0 w-fit
					flex-col justify-between flex dark:bg-gray-800 border-r dark:border-gray-700"
			style="height: calc(100vh - 4rem);"
		>
			<span class="p-8 text-center">
				<Heading tag="h1">ARSIS</Heading>
			</span>
			<div>
				<Sidebar />
			</div>
			<div class="flex-grow" />
			<div class="p-4">
				<DarkMode />
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
				class="absolute right-0 p-5 bg-white dark:bg-gray-900 overflow-y-auto h-full border-l"
				style="width: 18rem; height: calc(100vh - 4rem);"
			>
				{#each new Set([...Object.keys($keepables), ...Object.keys($graphdata)]) as label}
					<div class="border-b flex-row flex pb-2 mb-1 dark:border-gray-700">
						<Heading tag="h4">{label}</Heading>
						<FileEditSolid class="dark:text-gray-400 mr-2 h-7 text-gray-800" href="/rover" />
						<TrashBinOutline class="dark:text-gray-400 h-7 text-gray-800" />
					</div>

					{#if $keepables[label]}
						<div class="flex justify-left pr-2 pl-2 flex-wrap">
							{#each $keepables[label] as item}
								<div class="p-1">
									<Button
										color="alternative"
										on:click={() => keepables.removeElement(label, item[0])}
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
								on:click={() => graphdata.removeGraph(label, graph)}
								role="button"
								tabindex="0"
								on:keydown={() => graphdata.removeGraph(label, graph)}
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
