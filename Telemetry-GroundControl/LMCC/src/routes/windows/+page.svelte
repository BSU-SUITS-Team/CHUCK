<script lang="ts">
	import { Badge, Button } from 'flowbite-svelte';
	import { CloseOutline, PlayOutline, WindowOutline } from 'flowbite-svelte-icons';
	import {
		closeHololensWindow,
		hololensWindows,
		openHololensWindow,
		type HololensWindowOption
	} from '$lib/hololens';

	let pendingCommand = '';
	let lastCommand = '';
	let commandError = '';

	async function sendWindowCommand(action: 'open' | 'close', windowOption: HololensWindowOption) {
		pendingCommand = `${action}:${windowOption.id}`;
		commandError = '';

		try {
			if (action === 'open') {
				await openHololensWindow(windowOption.id);
			} else {
				await closeHololensWindow(windowOption.id);
			}
			lastCommand = `${action === 'open' ? 'Opened' : 'Closed'} ${windowOption.label}`;
		} catch (error) {
			commandError = error instanceof Error ? error.message : String(error);
		} finally {
			pendingCommand = '';
		}
	}
</script>

<svelte:head>
	<title>Hololens Windows</title>
</svelte:head>

<div class="h-full p-4 text-gray-900 dark:text-gray-100">
	<div class="mb-4 flex flex-wrap items-center justify-between gap-3">
		<div>
			<h2 class="text-2xl font-semibold leading-tight">Hololens Windows</h2>
			<div class="mt-2 flex flex-wrap gap-2">
				<Badge color="gray">{hololensWindows.length} windows</Badge>
				{#if lastCommand}
					<Badge color="green">{lastCommand}</Badge>
				{/if}
				{#if commandError}
					<Badge color="red">Command failed</Badge>
				{/if}
			</div>
		</div>
	</div>

	<div
		class="overflow-x-auto rounded border border-gray-200 bg-white dark:border-gray-700 dark:bg-gray-900"
	>
		<div class="min-w-[52rem]">
			<div
				class="grid grid-cols-[minmax(14rem,1fr)_minmax(16rem,1.25fr)_10rem_10rem] gap-3 border-b border-gray-200 bg-gray-50 px-4 py-2 text-xs font-semibold uppercase text-gray-500 dark:border-gray-700 dark:bg-gray-800 dark:text-gray-400"
			>
				<span>Window</span>
				<span>Purpose</span>
				<span>Open</span>
				<span>Close</span>
			</div>

			{#each hololensWindows as windowOption}
				<div
					class="grid grid-cols-[minmax(14rem,1fr)_minmax(16rem,1.25fr)_10rem_10rem] items-center gap-3 border-b border-gray-100 px-4 py-3 text-sm last:border-b-0 dark:border-gray-800"
				>
					<div class="flex items-center gap-3 font-medium">
						<WindowOutline class="h-5 w-5 text-gray-500 dark:text-gray-400" />
						<span>{windowOption.label}</span>
					</div>
					<p class="text-gray-600 dark:text-gray-400">{windowOption.description}</p>
					<Button
						size="sm"
						color="alternative"
						disabled={pendingCommand === `open:${windowOption.id}`}
						onclick={() => sendWindowCommand('open', windowOption)}
					>
						<PlayOutline class="mr-2 h-4 w-4" />
						Open
					</Button>
					<Button
						size="sm"
						color="alternative"
						disabled={pendingCommand === `close:${windowOption.id}`}
						onclick={() => sendWindowCommand('close', windowOption)}
					>
						<CloseOutline class="mr-2 h-4 w-4" />
						Close
					</Button>
				</div>
			{/each}
		</div>
	</div>
</div>
