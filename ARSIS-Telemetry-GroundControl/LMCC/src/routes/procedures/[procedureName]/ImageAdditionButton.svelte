<script lang="ts">
	import { ImageOutline, PlusSolid, TextSizeOutline, TrashBinOutline } from 'flowbite-svelte-icons';
	import type { MouseEventHandler } from 'svelte/elements';

	export let onclick: MouseEventHandler<EventTarget>;
	export let first = false;
	export let last = false;
	export let isImage = false;

	let offset = '-translate-y-1';
	if (first) {
		offset = 'translate-y-1.5';
	}
	if (last) {
		offset = '-translate-y-1/2';
	}
</script>

<button class="absolute right-8 -translate-y-8 -translate-x-1/2 p-1.5" on:click={onclick}>
	{#if !isImage}
		<ImageOutline class="w-3 h-3 text-gray-700" />
	{:else}
		<TextSizeOutline class="w-3 h-3 text-gray-700" />
	{/if}
</button>
