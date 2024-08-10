<script lang="ts">
	import type { Bounds, Range, Telemetry } from '$lib/biometrics';
	import { Card } from 'flowbite-svelte';

	export let name: string = 'Gauge';
	export let formatter: Function = (value: number) => {
		return value;
	};
	export let value: number;
	export let bounds: Range;

	const [low, high] = bounds.limit;

	const clamp = (value: number) => {
		return Math.max(0, Math.min(value, 100));
	};

	const setPercent = (start: number, end: number) => {
		return clamp((Math.abs(start - end) / high) * 100);
	};

	const maxHeight = `--max-height:${setPercent(bounds.max, high)}%;`; // from bounds.max to high
	const minHeight = `--min-height:${setPercent(bounds.min, low)}%;`; // from low to bounds.min

	$: position = `--position:${setPercent(low, value)}%;`; // from low to value
</script>

<Card>
	<div id="container" class="bg-green-500">
		<div id="max" style={maxHeight} class="bg-red-500" />
		<div id="min" style={minHeight} class="bg-red-500" />
		<div id="label" style={position} class="bg-white text-black dark:bg-black dark:text-white">
			{formatter(value)}
			{bounds.units}
		</div>
	</div>
	<p class="text-sm font-medium text-gray-900 dark:text-white">
		{name}
	</p>
</Card>

<style>
	#label {
		position: absolute;
		width: 100%;
		bottom: var(--position, 50%);
		justify-content: center;
	}

	#container {
		position: relative;
		height: 210px;
		border-radius: 15px;
	}

	#max {
		border-radius: 15px 15px 0 0;
		width: 100%;
		position: absolute;
		top: 0;
		height: var(--max-height, 25%);
	}

	#min {
		border-radius: 0 0 15px 15px;
		width: 100%;
		position: absolute;
		bottom: 0;
		height: var(--min-height, 25%);
	}
</style>
