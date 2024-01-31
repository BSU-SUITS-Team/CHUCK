<script>
	import { Button, Heading } from 'flowbite-svelte';
	import { notifications } from './store.js';

	function doProblem() {
		notifications.addNotification('Oxygen Tank Has Exploaded', 'error');
	}
	function doStatus() {
		notifications.addNotification('It is Fall Right Now', 'info');
	}
	function doNotify() {
		notifications.addNotification('Rocket Fell Over', 'warn');
	}
</script>

<svelte:head>
	<title>Home Page</title>
	<meta name="description" content="ARSIS ground station" />
</svelte:head>

<section>
	<div class="justify-center pr-72 mb-5">
		<Heading tag="h2">TMP Utils for Notifications</Heading>
	</div>
	<div class="justify-center pr-72 mb-2">
		<Button color="alternative" on:click={doProblem}>Show Error</Button>
	</div>
	<div class="justify-center pr-72 mb-2">
		<Button color="alternative" on:click={doNotify}>Show Warning</Button>
	</div>
	<div class="justify-center pr-72">
		<Button color="alternative" on:click={doStatus}>Show Info</Button>
	</div>
</section>

<style>
	section {
		display: flex;
		flex-direction: column;
		justify-content: center;
		align-items: center;
		flex: 0.6;
	}
</style>
