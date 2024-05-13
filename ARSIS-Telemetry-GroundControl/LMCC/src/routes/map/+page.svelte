<script lang="ts">
	import Panview from './Panview.svelte';
	import img from '$lib/images/rockyard_map_geo.jpg';
</script>

<Panview
	image={img}
	initalSize={3}
	minScale={0.6}
	maxScale={20}
	xRange={[-1000, 0]}
	yRange={[-1000, 0]}
	initalPosition={[-1300, -900]}
/>
