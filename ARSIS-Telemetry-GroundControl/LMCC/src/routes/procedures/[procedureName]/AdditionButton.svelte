<script lang="ts">
	import { PlusSolid } from "flowbite-svelte-icons";
	import type { MouseEventHandler } from "svelte/elements";

	export let onclick: MouseEventHandler<EventTarget>;
</script>

<button
	class="absolute left-1/2 -translate-y-1/2 -translate-x-1/2 bg-white rounded-full p-1.5 border border-gray-300"
	on:click={onclick}
>
	<PlusSolid class="w-3 h-3 text-gray-700" />
</button>
