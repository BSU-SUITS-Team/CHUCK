<script lang="ts">
	import { createWebSocketStore } from '$lib/datastore';
	let remoteData = createWebSocketStore('ws://localhost:8181/ws/events');

	remoteData.subscribe(console.log);
</script>

{JSON.stringify($remoteData)}

{#if $remoteData}
	<div>
		Time = {$remoteData.time}
	</div>
{/if}
