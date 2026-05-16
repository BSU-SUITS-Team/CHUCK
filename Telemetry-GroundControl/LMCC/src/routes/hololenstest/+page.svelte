<script>
	import { Input } from 'flowbite-svelte';

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
<!-- svelte-ignore a11y_media_has_caption -->
<video
	src="http://{ipaddr}/api/holographic/stream/live_high.mp4"
	bind:this={videoElement}
	controls
	trackSrc="live_high.mp4"
></video>
