<script lang="ts">
	import {
		TableHead,
		TableHeadCell,
		TableBody,
		TableBodyRow,
		TableBodyCell,
		Table,
		Tabs,
		TabItem,
		Card,
		Listgroup
	} from 'flowbite-svelte';
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

	$: currentTelemetry = telemetry[telemetry.length - 1];
	$: currentAstro =
		getEVA(currentTelemetry, selectedAstro) ??
		getEVA(currentTelemetry, getAstronauts(currentTelemetry)[0]);

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

	const categories = (eva: Astronaut | undefined) => {
		if (!eva) return {};

		return {
			'Suit Resources': suitResources(eva),
			'Suit Atmosphere': suitAtmosphere(eva),
			'Suit Helmet Fan': suitHelmet(eva),
			'Suit CO2 Scrubbers': suitScrubber(eva),
			'Suit Temperature': suitTemperature(eva)
		};
	};
</script>

<div class="h-full mr-2 ml-2 pt-2">
	<Tabs>
		{#each getAstronauts(currentTelemetry) as astro}
			<TabItem
				open
				title={astro}
				on:click={() => {
					selectedAstro = astro;
				}}
			>
				<div class="flex gap-2 flex-wrap">
					{#each Object.keys(categories(getEVA(currentTelemetry, astro))) as category}
						<Card>
							<div class="flex justify-between items-center mb-4">
								<h5 class="text-xl font-bold leading-none text-gray-900 dark:text-white">
									{category}
								</h5>
							</div>
							<Listgroup
								items={categories(getEVA(currentTelemetry, astro))[category]}
								let:item
								class="border-0 dark:!bg-transparent"
							>
								<div class="flex items-center space-x-4 rtl:space-x-reverse">
									<div class="flex-1 min-w-0">
										<p class="text-sm font-medium text-gray-900 dark:text-white">
											{item['key']}
										</p>
									</div>
									<div class="flex-1 min-w-0">
										<p class="text-sm font-medium text-gray-900 dark:text-white">
											{item['formatter'](item['value'])}
											{item['units']}
										</p>
									</div>
								</div>
							</Listgroup>
						</Card>
					{/each}
				</div>
			</TabItem>
		{/each}
	</Tabs>
	{#if currentAstro}
		<div class="pt-2">
			<Tabs>
				{#each Object.keys(categories(currentAstro)) as category}
					<TabItem open title={category}>
						<div class="flex gap-2 flex-wrap">
							{#each categories(currentAstro)[category] as data}
								<Gauge
									name={data.key}
									value={data.value}
									bounds={data.range}
									formatter={data.formatter}
								/>
							{/each}
						</div>
					</TabItem>
				{/each}
			</Tabs>
		</div>
	{/if}
</div>
