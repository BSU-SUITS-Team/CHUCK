<script lang="ts">
	import { page } from '$app/stores';
	import {
		getAstronauts,
		getEVA,
		getTelemetryEvent,
		sampleTelemetry,
		type Telemetry
	} from '$lib/biometrics';
	import {
		flattenMetricCategories,
		metricCategories,
		metricStatus,
		type FlatMetric
	} from '$lib/biometricMetrics';
	import { datastore } from '$lib/datastore';
	import { formatTime } from '$lib/formatting';
	import { onDestroy } from 'svelte';

	let telemetry: Telemetry[] = [sampleTelemetry];
	const unsubscribe = datastore.subscribe((store) => {
		telemetry = store['telemetry'] ?? [sampleTelemetry];
	});
	onDestroy(unsubscribe);

	const metricsFor = (evaKey: string): FlatMetric[] =>
		flattenMetricCategories(metricCategories(getEVA(currentTelemetry, evaKey)));

	const normalizeEvaParam = (value: string | null, availableAstronauts: string[]) => {
		const normalizedValue = value?.toLowerCase().replace(/[^a-z0-9]/g, '');
		const fallbackEva = availableAstronauts.includes('eva1')
			? 'eva1'
			: (availableAstronauts[0] ?? '');

		if (normalizedValue === 'all' || normalizedValue === 'both') return 'all';
		if (normalizedValue === '1' || normalizedValue === 'eva1') return 'eva1';
		if (normalizedValue === '2' || normalizedValue === 'eva2') return 'eva2';

		return fallbackEva;
	};

	const nominalCount = (metrics: FlatMetric[]) =>
		metrics.filter((metric) => metricStatus(metric) === 'nominal').length;

	const statusClass = (metric: FlatMetric) => metricStatus(metric);

	const elapsedTime = (event: Telemetry) => {
		const values = getAstronauts(event)
			.map((evaKey) => getEVA(event, evaKey)?.eva_elapsed_time)
			.filter((value): value is number => Number.isFinite(value));

		if (!values.length) return undefined;
		return Math.max(...values);
	};

	$: currentTelemetry = telemetry[telemetry.length - 1] ?? sampleTelemetry;
	$: telemetryEvent = getTelemetryEvent(currentTelemetry);
	$: astronauts = getAstronauts(currentTelemetry);
	$: availableAstronauts = astronauts.length ? astronauts : getAstronauts(sampleTelemetry);
	$: selectedEva = normalizeEvaParam($page.url.searchParams.get('eva'), availableAstronauts);
	$: visibleAstronauts =
		selectedEva === 'all'
			? availableAstronauts
			: availableAstronauts.includes(selectedEva)
				? [selectedEva]
				: availableAstronauts.slice(0, 1);
	$: maxElapsedTime = elapsedTime(currentTelemetry);
	$: telemetryTime = typeof telemetryEvent.time === 'number' ? telemetryEvent.time : undefined;
</script>

<svelte:head>
	<title>Low Res Biometrics</title>
	<meta name="description" content="Read-only live biometrics view for displays under 1000px wide" />
</svelte:head>

<div
	class="low-res-shell"
	data-testid="low-res-biometrics"
	style={`--eva-count: ${Math.max(visibleAstronauts.length, 1)}`}
>
	<header class="status-strip">
		<div class="brand">
			<span class="title">LMCC BIOMETRICS</span>
		</div>
		<div class="status-line">
			<span class:live={$datastore.connected} class="live-dot"></span>
			<span>{$datastore.connected ? 'LIVE' : 'CONNECTING'}</span>
			{#if maxElapsedTime !== undefined}
				<span>T+{formatTime(maxElapsedTime)}</span>
			{/if}
			{#if telemetryTime !== undefined}
				<span>TS {telemetryTime}</span>
			{/if}
		</div>
	</header>

	<main class="eva-grid">
		{#each visibleAstronauts as evaKey}
			{@const metrics = metricsFor(evaKey)}
			<section class="eva-panel" aria-label={`${evaKey.toUpperCase()} live biometrics`}>
				<div class="eva-header">
					<h1>{evaKey.toUpperCase()}</h1>
					<span>{nominalCount(metrics)}/{metrics.length} NOM</span>
				</div>

				<div class="metric-grid">
					{#each metrics as metric}
						<div class={`metric-row ${statusClass(metric)}`}>
							<span class="category">{metric.categoryCode}</span>
							<span class="label">{metric.shortKey}</span>
							<span class="value">{metric.formatter(metric.value)}</span>
							{#if metric.units}
								<span class="unit">{metric.units}</span>
							{/if}
						</div>
					{/each}
				</div>
			</section>
		{/each}
	</main>
</div>

<style>
	:global(html:has(.low-res-shell)),
	:global(body:has(.low-res-shell)) {
		overflow: hidden;
	}

	.low-res-shell {
		box-sizing: border-box;
		display: grid;
		grid-template-rows: 1.75rem minmax(0, 1fr);
		gap: 0.3rem;
		width: 100vw;
		height: 100vh;
		overflow: hidden;
		padding: 0.35rem;
		background: #f1f5f9;
		color: #111827;
		font-family:
			ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, 'Liberation Mono', 'Courier New',
			monospace;
		letter-spacing: 0;
	}

	.status-strip,
	.eva-panel {
		min-width: 0;
		border: 1px solid #94a3b8;
		background: #ffffff;
	}

	.status-strip {
		display: flex;
		align-items: center;
		justify-content: space-between;
		gap: 0.5rem;
		padding: 0 0.45rem;
		overflow: hidden;
	}

	.brand,
	.status-line {
		display: flex;
		min-width: 0;
		align-items: center;
		gap: 0.4rem;
		white-space: nowrap;
	}

	.title {
		font-size: 0.86rem;
		font-weight: 800;
		line-height: 1;
	}

	.status-line {
		font-size: 0.66rem;
		font-weight: 700;
		line-height: 1;
		color: #475569;
	}

	.live-dot {
		width: 0.45rem;
		height: 0.45rem;
		flex: 0 0 auto;
		border-radius: 999px;
		background: #f59e0b;
	}

	.live-dot.live {
		background: #16a34a;
	}

	.eva-grid {
		display: grid;
		grid-template-columns: repeat(var(--eva-count), minmax(0, 1fr));
		gap: 0.3rem;
		min-height: 0;
		overflow: hidden;
	}

	.eva-panel {
		display: grid;
		grid-template-rows: 1.7rem minmax(0, 1fr);
		min-height: 0;
		overflow: hidden;
	}

	.eva-header {
		display: flex;
		align-items: center;
		justify-content: space-between;
		gap: 0.35rem;
		min-width: 0;
		border-bottom: 1px solid #cbd5e1;
		background: #e2e8f0;
		padding: 0 0.35rem;
	}

	.eva-header h1 {
		overflow: hidden;
		margin: 0;
		text-overflow: ellipsis;
		white-space: nowrap;
		font-size: 0.92rem;
		font-weight: 800;
		line-height: 1;
	}

	.eva-header span {
		flex: 0 0 auto;
		font-size: 0.64rem;
		font-weight: 800;
		line-height: 1;
		color: #334155;
	}

	.metric-grid {
		display: grid;
		grid-template-columns: repeat(2, minmax(0, 1fr));
		align-content: start;
		gap: 0.12rem 0.22rem;
		min-height: 0;
		overflow: hidden;
		padding: 0.22rem;
	}

	.metric-row {
		display: grid;
		grid-template-columns: 1.4rem minmax(0, 1fr) max-content max-content;
		align-items: center;
		gap: 0.18rem;
		height: 1.16rem;
		min-width: 0;
		overflow: hidden;
		border-left: 3px solid #16a34a;
		background: #f8fafc;
		padding: 0 0.18rem;
		font-size: 0.66rem;
		line-height: 1;
	}

	.metric-row.low {
		border-left-color: #d97706;
		background: #fffbeb;
	}

	.metric-row.high {
		border-left-color: #dc2626;
		background: #fef2f2;
	}

	.category,
	.unit {
		overflow: hidden;
		color: #64748b;
		font-size: 0.53rem;
		font-weight: 800;
		line-height: 1;
		text-overflow: clip;
		white-space: nowrap;
	}

	.label,
	.value {
		min-width: 0;
		overflow: hidden;
		line-height: 1;
		white-space: nowrap;
	}

	.label {
		text-overflow: ellipsis;
		font-weight: 700;
		color: #1f2937;
	}

	.value {
		text-align: right;
		font-weight: 900;
		color: #020617;
	}

	.unit {
		text-align: left;
	}

	@media (max-width: 700px) {
		.low-res-shell {
			padding: 0.25rem;
		}
	}
</style>
