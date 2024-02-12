<script lang="ts">
	import Panview from './Panview.svelte';
	import img from '$lib/images/test.svg';
</script>

<Panview
	image={img}
	initalSize={3}
	minScale={2}
	maxScale={20}
	xRange={[-1000, 0]}
	yRange={[-1000, 0]}
	initalPosition={[-1300, -900]}
/>
