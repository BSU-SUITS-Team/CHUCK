<script lang="ts">
	import { TrashBinOutline } from 'flowbite-svelte-icons';
	import type { MouseEventHandler } from 'svelte/elements';

	export let onclick: MouseEventHandler<EventTarget>;
</script>

<button class="absolute right-1 -translate-y-8 -translate-x-1/2 p-1.5" on:click={onclick}>
	<TrashBinOutline class="w-3 h-3 text-gray-700" />
</button>
