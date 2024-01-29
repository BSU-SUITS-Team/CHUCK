<script>
	import { MapPinAltSolid } from 'flowbite-svelte-icons';

	export let color = 'primary';
	export let onclick = () => {};
	export let move = false;
	export let bright = true;
</script>

<div>
	<MapPinAltSolid 
		on:click={onclick}
		class="m-2 w-5 h-5 text-{color}-{bright ? '500' : '100'}
		transition duration-75 focus:outline-none"
		style="{move ? 'transform: translate3d(-95%, -150%, 0);' : ''}"
	/>
</div>
