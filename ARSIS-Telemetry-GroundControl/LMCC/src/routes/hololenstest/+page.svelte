<script>
	import { Input, Video } from 'flowbite-svelte';


	let ipaddr = '192.168.125.50';
	let videoElement;

	$: {
		if (videoElement) {
			videoElement.load();
			videoElement.play();
		}
	}
</script>

<Input bind:value={ipaddr} />
<video
	src="http://{ipaddr}/api/holographic/stream/live_high.mp4"
	controls
	trackSrc="live_high.mp4"
/>
