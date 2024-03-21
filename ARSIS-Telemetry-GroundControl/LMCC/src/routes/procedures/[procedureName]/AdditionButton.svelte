<script lang="ts">
	import { PlusSolid } from "flowbite-svelte-icons";
	import type { MouseEventHandler } from "svelte/elements";

	export let onclick: MouseEventHandler<EventTarget>;
	export let first: false;
	export let last: false;

	let offset = "-translate-y-1"
	if (first) {
		offset = "translate-y-1.5"
	}
	if (last) {
		offset = "-translate-y-1/2"
	}
	

</script>

<button
	class="absolute left-1/2 {offset} -translate-x-1/2 bg-white rounded-full p-1.5 border border-gray-300"
	on:click={onclick}
>
	<PlusSolid class="w-2 h-2 text-gray-700" />
</button>
