<script>
	import { Card, Heading, Spinner } from 'flowbite-svelte';

	export let name = 'TestCameraFeed';
</script>

<div role="button">
	<Card>
		<Heading tag="h5" class="mb-5">{name}</Heading>
		<div
			class="text-center w-80 h-52 bg-gray-200 rounded-lg flex items-center justify-center dark:bg-gray-700"
		>
			<Spinner />
		</div>
	</Card>
</div>
