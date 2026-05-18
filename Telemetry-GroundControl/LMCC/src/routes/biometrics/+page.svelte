<script lang="ts">
	import {
		sampleTelemetry,
		getAstronauts,
		getEVA,
		ResourceBounds,
		HelmetBounds,
		AtmosphereBounds,
		ScrubberBounds,
		TemperatureBounds,
		type Astronaut,
		type Range,
		type Telemetry
	} from '$lib/biometrics';

	import { datastore } from '$lib/datastore';
	import { onDestroy } from 'svelte';
	import Gauge from './gauge.svelte';
	import { formatTime, formatDecimals } from '$lib/formatting';

	let selectedAstro = 'eva2';
	let selectedCategory = 'Suit Resources';

	type Metric = {
		key: string;
		units: string;
		formatter: (value: number) => string | number;
		range: Range;
		value: number;
	};

	let telemetry: Telemetry[] = [sampleTelemetry];
	const unsubscribe = datastore.subscribe((store) => {
		telemetry = store['telemetry'] ?? [sampleTelemetry];
	});
	onDestroy(unsubscribe);

	const suitResources = (eva: Astronaut): Metric[] => {
		const resources: Metric[] = [
			{
				key: 'EVA Elapsed Time',
				units: ResourceBounds.eva_elapsed_time.units,
				formatter: formatTime,
				range: ResourceBounds.eva_elapsed_time,
				value: eva.eva_elapsed_time
			}
		];

		if (typeof eva.primary_battery_level === 'number') {
			resources.push({
				key: 'Primary Battery Level',
				units: ResourceBounds.primary_battery_level.units,
				formatter: formatDecimals(2),
				range: ResourceBounds.primary_battery_level,
				value: eva.primary_battery_level
			});
		}

		if (typeof eva.secondary_battery_level === 'number') {
			resources.push({
				key: 'Secondary Battery Level',
				units: ResourceBounds.secondary_battery_level.units,
				formatter: formatDecimals(2),
				range: ResourceBounds.secondary_battery_level,
				value: eva.secondary_battery_level
			});
		}

		if (typeof eva.battery_level === 'number') {
			resources.push({
				key: 'Battery Level',
				units: ResourceBounds.battery_level.units,
				formatter: formatDecimals(2),
				range: ResourceBounds.battery_level,
				value: eva.battery_level
			});
		}

		return [
			...resources,
			{
				key: 'Primary Oxygen Storage',
				units: ResourceBounds.oxy_pri_storage.units,
				formatter: formatDecimals(2),
				range: ResourceBounds.oxy_pri_storage,
				value: eva.oxy_pri_storage
			},
			{
				key: 'Secondary Oxygen Storage',
				units: ResourceBounds.oxy_sec_storage.units,
				formatter: formatDecimals(2),
				range: ResourceBounds.oxy_sec_storage,
				value: eva.oxy_sec_storage
			},
			{
				key: 'Primary Oxygen Pressure',
				units: ResourceBounds.oxy_pri_pressure.units,
				formatter: formatDecimals(2),
				range: ResourceBounds.oxy_pri_pressure,
				value: eva.oxy_pri_pressure
			},
			{
				key: 'Secondary Oxygen Pressure',
				units: ResourceBounds.oxy_sec_pressure.units,
				formatter: formatDecimals(2),
				range: ResourceBounds.oxy_sec_pressure,
				value: eva.oxy_sec_pressure
			},
			{
				key: 'Coolant Storage',
				units: ResourceBounds.coolant_storage.units,
				formatter: formatDecimals(2),
				range: ResourceBounds.coolant_storage,
				value: eva.coolant_storage
			}
		];
	};

	const suitAtmosphere = (eva: Astronaut): Metric[] => {
		return [
			{
				key: 'Heart Rate',
				units: AtmosphereBounds.heart_rate.units,
				formatter: formatDecimals(2),
				range: AtmosphereBounds.heart_rate,
				value: eva.heart_rate
			},
			{
				key: 'Oxygen Consumption',
				units: AtmosphereBounds.oxy_consumption.units,
				formatter: formatDecimals(2),
				range: AtmosphereBounds.oxy_consumption,
				value: eva.oxy_consumption
			},
			{
				key: 'CO2 Production',
				units: AtmosphereBounds.co2_production.units,
				formatter: formatDecimals(2),
				range: AtmosphereBounds.co2_production,
				value: eva.co2_production
			},
			{
				key: 'Suit Pressure Oxygen',
				units: AtmosphereBounds.suit_pressure_oxy.units,
				formatter: formatDecimals(2),
				range: AtmosphereBounds.suit_pressure_oxy,
				value: eva.suit_pressure_oxy
			},
			{
				key: 'Suit Pressure CO2',
				units: AtmosphereBounds.suit_pressure_co2.units,
				formatter: formatDecimals(2),
				range: AtmosphereBounds.suit_pressure_co2,
				value: eva.suit_pressure_co2
			},
			{
				key: 'Suit Pressure Other',
				units: AtmosphereBounds.suit_pressure_other.units,
				formatter: formatDecimals(2),
				range: AtmosphereBounds.suit_pressure_other,
				value: eva.suit_pressure_other
			},
			{
				key: 'Suit Pressure Total',
				units: AtmosphereBounds.suit_pressure_total.units,
				formatter: formatDecimals(2),
				range: AtmosphereBounds.suit_pressure_total,
				value: eva.suit_pressure_total
			},
			{
				key: 'Helmet Pressure CO2',
				units: AtmosphereBounds.helmet_pressure_co2.units,
				formatter: formatDecimals(2),
				range: AtmosphereBounds.helmet_pressure_co2,
				value: eva.helmet_pressure_co2
			}
		];
	};

	const suitHelmet = (eva: Astronaut): Metric[] => {
		return [
			{
				key: 'Primary Fan Speed',
				units: HelmetBounds.fan_pri_rpm.units,
				formatter: formatDecimals(2),
				range: HelmetBounds.fan_pri_rpm,
				value: eva.fan_pri_rpm
			},
			{
				key: 'Secondary Fan Speed',
				units: HelmetBounds.fan_sec_rpm.units,
				formatter: formatDecimals(2),
				range: HelmetBounds.fan_sec_rpm,
				value: eva.fan_sec_rpm
			}
		];
	};

	const suitScrubber = (eva: Astronaut): Metric[] => {
		return [
			{
				key: 'Scrubber A CO2 Storage',
				units: ScrubberBounds.scrubber_a_co2_storage.units,
				formatter: formatDecimals(2),
				range: ScrubberBounds.scrubber_a_co2_storage,
				value: eva.scrubber_a_co2_storage
			},
			{
				key: 'Scrubber B CO2 Storage',
				units: ScrubberBounds.scrubber_b_co2_storage.units,
				formatter: formatDecimals(2),
				range: ScrubberBounds.scrubber_b_co2_storage,
				value: eva.scrubber_b_co2_storage
			}
		];
	};

	const suitTemperature = (eva: Astronaut): Metric[] => {
		return [
			{
				key: 'Temperature',
				units: TemperatureBounds.temperature.units,
				formatter: formatDecimals(2),
				range: TemperatureBounds.temperature,
				value: eva.temperature
			},
			{
				key: 'Coolant Gas Pressure',
				units: TemperatureBounds.coolant_gas_pressure.units,
				formatter: formatDecimals(2),
				range: TemperatureBounds.coolant_gas_pressure,
				value: eva.coolant_gas_pressure
			},
			{
				key: 'Coolant Liquid Pressure',
				units: TemperatureBounds.coolant_liquid_pressure.units,
				formatter: formatDecimals(2),
				range: TemperatureBounds.coolant_liquid_pressure,
				value: eva.coolant_liquid_pressure
			}
		];
	};

	type CategoryMap = Record<string, Metric[]>;

	const categories = (eva: Astronaut | undefined): CategoryMap => {
		if (!eva) return {};

		return {
			'Suit Resources': suitResources(eva),
			'Suit Atmosphere': suitAtmosphere(eva),
			'Suit Helmet Fan': suitHelmet(eva),
			'Suit CO2 Scrubbers': suitScrubber(eva),
			'Suit Temperature': suitTemperature(eva)
		};
	};

	$: currentTelemetry = telemetry[telemetry.length - 1];
	$: astronauts = getAstronauts(currentTelemetry);
	$: selectedAstroKey = astronauts.includes(selectedAstro) ? selectedAstro : (astronauts[0] ?? '');
	$: currentAstro =
		getEVA(currentTelemetry, selectedAstroKey) ??
		getEVA(currentTelemetry, getAstronauts(currentTelemetry)[0]);
	$: categoryMap = categories(currentAstro);
	$: categoryNames = Object.keys(categoryMap);
	$: if (categoryNames.length && !categoryNames.includes(selectedCategory)) {
		selectedCategory = categoryNames[0];
	}
	$: selectedMetrics = categoryMap[selectedCategory] ?? [];
	$: selectedNominal = selectedMetrics.filter((metric) => metricStatus(metric) === 'nominal').length;

	const clampPercent = (input: number) => Math.max(0, Math.min(input, 100));

	const metricPercent = (metric: Metric, value = metric.value) => {
		const [low, high] = metric.range.limit;
		const span = high - low;
		if (!Number.isFinite(span) || span === 0) return 0;
		return clampPercent(((value - low) / span) * 100);
	};

	const metricStyle = (metric: Metric) => {
		return `--value:${metricPercent(metric)}%; --min:${metricPercent(metric, metric.range.min)}%; --max:${metricPercent(metric, metric.range.max)}%;`;
	};

	const metricStatus = (metric: Metric) => {
		if (metric.value < metric.range.min) return 'low';
		if (metric.value > metric.range.max) return 'high';
		return 'nominal';
	};

	const issueCount = (metrics: Metric[]) =>
		metrics.filter((metric) => metricStatus(metric) !== 'nominal').length;

	const astronautButtonClass = (active: boolean) =>
		`rounded-md px-3 py-1.5 text-sm font-semibold transition ${
			active
				? 'bg-slate-900 text-white shadow-sm dark:bg-white dark:text-slate-950'
				: 'text-slate-600 hover:bg-slate-100 hover:text-slate-950 dark:text-slate-300 dark:hover:bg-slate-800 dark:hover:text-white'
		}`;

	const categoryButtonClass = (active: boolean) =>
		`shrink-0 rounded-md border px-3 py-1.5 text-sm font-semibold transition ${
			active
				? 'border-sky-500 bg-sky-50 text-sky-700 dark:border-sky-400 dark:bg-sky-500/15 dark:text-sky-200'
				: 'border-slate-200 bg-white text-slate-600 hover:border-slate-300 hover:bg-slate-50 hover:text-slate-950 dark:border-slate-700 dark:bg-slate-900 dark:text-slate-300 dark:hover:border-slate-600 dark:hover:bg-slate-800 dark:hover:text-white'
		}`;

	const panelClass =
		'rounded-lg border border-slate-200 bg-white shadow-sm transition dark:border-slate-700 dark:bg-slate-900';

	const statusDotClass = (metric: Metric) => {
		const status = metricStatus(metric);
		return status === 'nominal'
			? 'bg-emerald-500'
			: status === 'low'
				? 'bg-amber-400'
				: 'bg-rose-500';
	};

	const statusTextClass = (metric: Metric) => {
		const status = metricStatus(metric);
		return status === 'nominal'
			? 'text-slate-950 dark:text-white'
			: status === 'low'
				? 'text-amber-700 dark:text-amber-300'
				: 'text-rose-700 dark:text-rose-300';
	};
</script>

<div class="h-full px-3 py-3 text-slate-900 dark:text-slate-100">
	{#if astronauts.length}
		<section
			class="overflow-hidden rounded-lg border border-slate-200 bg-slate-50/80 shadow-sm dark:border-slate-700 dark:bg-slate-950/40"
		>
			<div
				class="flex flex-col gap-3 border-b border-slate-200 bg-white px-4 py-3 dark:border-slate-700 dark:bg-slate-900 sm:flex-row sm:items-center sm:justify-between"
			>
				<div class="min-w-0">
					<p class="text-xs font-semibold uppercase text-slate-500 dark:text-slate-400">Biometrics</p>
					<div class="mt-1 flex min-w-0 items-center gap-2">
						<span class="h-2 w-2 rounded-full bg-emerald-500"></span>
						<h1 class="truncate text-lg font-semibold text-slate-950 dark:text-white">
							{selectedAstroKey.toUpperCase()}
						</h1>
						<span
							class="rounded-full bg-slate-100 px-2 py-0.5 text-xs font-medium text-slate-600 dark:bg-slate-800 dark:text-slate-300"
						>
							{categoryNames.length} groups
						</span>
					</div>
				</div>

				<div
					class="flex w-fit max-w-full gap-1 overflow-x-auto rounded-lg border border-slate-200 bg-slate-50 p-1 dark:border-slate-700 dark:bg-slate-950"
				>
					{#each astronauts as astro}
						<button
							type="button"
							class={astronautButtonClass(astro === selectedAstroKey)}
							onclick={() => {
								selectedAstro = astro;
							}}
						>
							{astro.toUpperCase()}
						</button>
					{/each}
				</div>
			</div>

			{#if currentAstro}
				<div class="border-b border-slate-200 px-4 py-3 dark:border-slate-700">
					<div class="flex gap-2 overflow-x-auto pb-1">
						{#each categoryNames as category}
							<button
								type="button"
								class={categoryButtonClass(category === selectedCategory)}
								onclick={() => {
									selectedCategory = category;
								}}
							>
								{category}
							</button>
						{/each}
					</div>
				</div>

				<div class="grid gap-3 p-3 xl:grid-cols-[minmax(0,1fr)_minmax(22rem,0.85fr)]">
					<div class="grid auto-rows-min gap-3 2xl:grid-cols-2">
						{#each categoryNames as category}
							<section class={panelClass}>
								<button
									type="button"
									class="flex w-full items-center justify-between gap-3 border-b border-slate-200 px-3 py-2 text-left dark:border-slate-700"
									onclick={() => {
										selectedCategory = category;
									}}
								>
									<span class="min-w-0 truncate text-sm font-semibold text-slate-950 dark:text-white">
										{category}
									</span>
									<span
										class={`shrink-0 rounded-full px-2 py-0.5 text-xs font-semibold ${
											issueCount(categoryMap[category])
												? 'bg-rose-500/15 text-rose-700 dark:text-rose-300'
												: 'bg-emerald-500/15 text-emerald-700 dark:text-emerald-300'
										}`}
									>
										{categoryMap[category].length}
									</span>
								</button>

								<div class="divide-y divide-slate-100 dark:divide-slate-800">
									{#each categoryMap[category] as item}
										<div class="px-3 py-2">
											<div class="flex items-center justify-between gap-3">
												<div class="flex min-w-0 items-center gap-2">
													<span class={`h-2 w-2 shrink-0 rounded-full ${statusDotClass(item)}`}></span>
													<p class="min-w-0 text-sm font-medium leading-tight text-slate-700 dark:text-slate-200">
														{item.key}
													</p>
												</div>
												<div class="shrink-0 text-right">
													<span class={`text-sm font-semibold tabular-nums ${statusTextClass(item)}`}>
														{item.formatter(item.value)}
													</span>
													{#if item.units}
														<span class="ml-1 text-[11px] font-medium text-slate-500 dark:text-slate-400">
															{item.units}
														</span>
													{/if}
												</div>
											</div>
											<div class="metric-track mt-1.5" style={metricStyle(item)}>
												<span class="metric-safe"></span>
												<span class="metric-fill"></span>
												<span class="metric-marker"></span>
											</div>
										</div>
									{/each}
								</div>
							</section>
						{/each}
					</div>

					<section
						class="min-w-0 rounded-lg border border-slate-200 bg-white shadow-sm dark:border-slate-700 dark:bg-slate-900"
					>
						<div
							class="flex items-center justify-between gap-3 border-b border-slate-200 px-3 py-2 dark:border-slate-700"
						>
							<div class="min-w-0">
								<h2 class="truncate text-sm font-semibold text-slate-950 dark:text-white">
									{selectedCategory}
								</h2>
								<p class="text-xs text-slate-500 dark:text-slate-400">
									{selectedMetrics.length} readings
								</p>
							</div>
							<span
								class="shrink-0 rounded-full bg-slate-100 px-2 py-0.5 text-xs font-semibold text-slate-600 dark:bg-slate-800 dark:text-slate-300"
							>
								{selectedNominal}/{selectedMetrics.length} nominal
							</span>
						</div>

						<div class="grid gap-2 p-3 sm:grid-cols-2 xl:grid-cols-1 2xl:grid-cols-2">
							{#each selectedMetrics as data}
								<Gauge
									name={data.key}
									value={data.value}
									bounds={data.range}
									formatter={data.formatter}
								/>
							{/each}
						</div>
					</section>
				</div>
			{/if}
		</section>
	{:else}
		<section
			class="rounded-lg border border-slate-200 bg-white p-4 text-sm text-slate-500 shadow-sm dark:border-slate-700 dark:bg-slate-900 dark:text-slate-400"
		>
			No telemetry received.
		</section>
	{/if}
</div>

<style>
	.metric-track {
		position: relative;
		height: 0.375rem;
		overflow: hidden;
		border-radius: 999px;
		background: rgb(244 63 94 / 0.45);
	}

	.metric-safe,
	.metric-fill,
	.metric-marker {
		position: absolute;
		top: 0;
		bottom: 0;
	}

	.metric-safe {
		left: var(--min);
		width: calc(var(--max) - var(--min));
		background: rgb(16 185 129);
	}

	.metric-fill {
		left: 0;
		width: var(--value);
		background: rgb(14 165 233 / 0.35);
	}

	.metric-marker {
		left: var(--value);
		width: 0.25rem;
		transform: translateX(-0.125rem);
		border-radius: 999px;
		background: rgb(15 23 42);
		box-shadow: 0 0 0 1px rgb(255 255 255 / 0.9);
	}

	:global(.dark) .metric-track {
		background: rgb(190 18 60 / 0.5);
	}

	:global(.dark) .metric-marker {
		background: rgb(226 232 240);
		box-shadow: 0 0 0 1px rgb(15 23 42 / 0.8);
	}
</style>
