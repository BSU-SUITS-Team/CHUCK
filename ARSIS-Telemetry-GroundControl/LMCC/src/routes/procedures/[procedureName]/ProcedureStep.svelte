<script lang="ts">
	import { Timeline, TimelineItem, Button, Textarea } from 'flowbite-svelte';
	import { PlusSolid } from 'flowbite-svelte-icons';

	export let title: string;
	export let editMode: boolean = false;
	export let date: string;
	export let description: string;
	export let links: Array<string> = [];
	export let problemLinks: Array<string> = [];
</script>

<TimelineItem {title} {date} class="pb-2">
	{#if editMode}
		<Textarea
			class="text-base font-normal text-gray-500 dark:text-gray-400"
			bind:value={description}
			rows="3"
		/>
		<br />
	{:else}
		<p class="text-base font-normal text-gray-500 dark:text-gray-400">
			{description}
		</p>
	{/if}
	{#each problemLinks as link}
		<Button class="m-1">{link}</Button>
	{/each}
	{#each links as link}
		<Button color="alternative" class="m-1">{link}</Button>
	{/each}

	<br />
	{#if editMode}
		<Button color="alternative" class="m-1">New Step<PlusSolid class="w-3 h-3 ml-2" /></Button>
		<Button color="alternative" class="m-1">Substep<PlusSolid class="w-3 h-3 ml-2" /></Button>
		<Button class="m-1">Remove</Button>
	{/if}
</TimelineItem>
