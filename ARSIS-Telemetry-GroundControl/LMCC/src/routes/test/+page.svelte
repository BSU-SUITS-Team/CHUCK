<script lang="ts">
	import { createWebSocketStore } from './teststore';
	let remoteData = createWebSocketStore('ws://localhost:8181/ws/events');
</script>

{JSON.stringify($remoteData)}

{#if $remoteData}
	<div>
		Time = {$remoteData.time}
	</div>
{/if}
