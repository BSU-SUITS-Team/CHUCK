<script lang="ts">
	import { Card, Heading, Spinner } from 'flowbite-svelte';

	export let name = 'TestCameraFeed';
	export let source = "https://mars.nasa.gov/system/resources/detail_files/25904_1-PIA24546-1200.jpg";
</script>

<div role="button">
	<Card>
		<Heading tag="h5" class="mb-5">{name}</Heading>
		<div
			class="text-center w-80 h-52 bg-gray-200 rounded-lg flex items-center justify-center dark:bg-gray-700"
		>
			<img
				src="{source}"
				alt=""
			/>
		</div>
	</Card>
</div>
