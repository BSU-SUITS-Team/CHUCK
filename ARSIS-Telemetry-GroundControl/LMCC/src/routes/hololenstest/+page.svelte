<script>
	import { Input, Video } from 'flowbite-svelte';

	let ipaddr = '192.168.137.75';

</script>

<Input bind:value={ipaddr} />
<Video
	src="http://{ipaddr}/api/holographic/stream/live_high.mp4"
	controls
	trackSrc="live_high.mp4"
/>
