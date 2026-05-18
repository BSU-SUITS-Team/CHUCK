<script lang="ts">
	import type { Range } from '$lib/biometrics';

	export let name: string = 'Gauge';
	export let formatter: Function = (value: number) => {
		return value;
	};
	export let value: number;
	export let bounds: Range;

	const clamp = (input: number) => {
		return Math.max(0, Math.min(input, 100));
	};

	const percentFor = (input: number) => {
		const [low, high] = bounds.limit;
		const span = high - low;
		if (!Number.isFinite(span) || span === 0) return 0;
		return clamp(((input - low) / span) * 100);
	};

	const compactNumber = (input: number) => {
		return Number.isInteger(input) ? input.toString() : input.toFixed(2).replace(/\.?0+$/, '');
	};

	$: valuePercent = percentFor(value);
	$: minPercent = percentFor(bounds.min);
	$: maxPercent = percentFor(bounds.max);
	$: nominalPercent = typeof bounds.nominal === 'number' ? percentFor(bounds.nominal) : undefined;
	$: status = value < bounds.min ? 'LOW' : value > bounds.max ? 'HIGH' : 'NOMINAL';
	$: statusClass =
		status === 'NOMINAL'
			? 'bg-emerald-500/15 text-emerald-700 dark:text-emerald-300'
			: 'bg-rose-500/15 text-rose-700 dark:text-rose-300';
	$: gaugeStyle = `--value:${valuePercent}%; --min:${minPercent}%; --max:${maxPercent}%; --nominal:${
		nominalPercent ?? valuePercent
	}%;`;
	$: formattedValue = formatter(value);
</script>

<div
	class="rounded-lg border border-slate-200 bg-slate-50/80 p-3 shadow-sm dark:border-slate-700 dark:bg-slate-800/70"
>
	<div class="mb-2 flex items-start justify-between gap-3">
		<p class="min-w-0 truncate text-sm font-semibold text-slate-900 dark:text-white">{name}</p>
		<span class={`shrink-0 rounded-full px-2 py-0.5 text-[11px] font-semibold ${statusClass}`}>
			{status}
		</span>
	</div>

	<div class="mb-2 flex items-baseline gap-1">
		<span class="text-2xl font-semibold tabular-nums text-slate-950 dark:text-white">
			{formattedValue}
		</span>
		{#if bounds.units}
			<span class="text-xs font-medium text-slate-500 dark:text-slate-400">{bounds.units}</span>
		{/if}
	</div>

	<div class="range-track" style={gaugeStyle}>
		<span class="range-safe"></span>
		<span class="range-fill"></span>
		{#if nominalPercent !== undefined}
			<span class="range-nominal"></span>
		{/if}
		<span class="range-marker"></span>
	</div>

	<div class="mt-1.5 flex justify-between text-[11px] tabular-nums text-slate-500 dark:text-slate-400">
		<span>{compactNumber(bounds.limit[0])}</span>
		<span>{compactNumber(bounds.min)}-{compactNumber(bounds.max)}</span>
		<span>{compactNumber(bounds.limit[1])}</span>
	</div>
</div>

<style>
	.range-track {
		position: relative;
		height: 0.5rem;
		overflow: hidden;
		border-radius: 999px;
		background: rgb(244 63 94 / 0.55);
	}

	.range-safe,
	.range-fill,
	.range-marker,
	.range-nominal {
		position: absolute;
		top: 0;
		bottom: 0;
	}

	.range-safe {
		left: var(--min);
		width: calc(var(--max) - var(--min));
		background: rgb(16 185 129);
	}

	.range-fill {
		left: 0;
		width: var(--value);
		background: rgb(14 165 233 / 0.3);
	}

	.range-marker {
		left: var(--value);
		width: 0.25rem;
		transform: translateX(-0.125rem);
		border-radius: 999px;
		background: rgb(15 23 42);
		box-shadow: 0 0 0 1px rgb(255 255 255 / 0.8);
	}

	.range-nominal {
		left: var(--nominal);
		width: 0.125rem;
		transform: translateX(-0.0625rem);
		background: rgb(255 255 255 / 0.85);
	}

	:global(.dark) .range-marker {
		background: rgb(226 232 240);
		box-shadow: 0 0 0 1px rgb(15 23 42 / 0.8);
	}

	:global(.dark) .range-track {
		background: rgb(190 18 60 / 0.5);
	}
</style>
