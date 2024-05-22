<script>
	import { goto } from '$app/navigation';

	import { onMount } from 'svelte';

	onMount(() => {
		goto('/procedures');
	});
</script>

<svelte:head>
	<title>Home Page</title>
	<meta name="description" content="ARSIS ground station" />
</svelte:head>

<section />
