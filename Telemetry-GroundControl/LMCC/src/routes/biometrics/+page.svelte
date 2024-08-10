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
		compareValueToBounds,
		ResourceBounds,
		HelmetBounds,
		AtmosphereBounds,
		ScrubberBounds,
		TemperatureBounds,
		Threshold,
		type Astronaut,
		type Telemetry
	} from '$lib/biometrics';

	import { datastore } from '$lib/datastore';
	import { onDestroy } from 'svelte';
	import Gauge from './gauge.svelte';
	import { formatTime, formatDecimals } from '$lib/formatting';

	let selectedAstro = 'eva2';

	const errorColor = 'p-2 text-align-left rounded-md text-red-500';
	const warningColor = 'p-2 text-align-left rounded-md text-orange-400';
	const goodColor = 'p-2 text-align-left rounded-md ';

	const colors = {
		[Threshold.Min]: errorColor,
		[Threshold.Nominal]: goodColor,
		[Threshold.Max]: errorColor
	};

	const getColor = (value, range) => {
		const threshold = compareValueToBounds(value, range);
		return colors[threshold];
	};

	let telemetry: Telemetry[];
	const unsubscribe = datastore.subscribe((store) => {
		let data = [sampleTelemetry];
		if (store['telemetry']) {
			const lower = JSON.stringify(store['telemetry']).toLowerCase();
			data = JSON.parse(lower);
		}
		telemetry = data;
	});
	onDestroy(unsubscribe);

	$: currentTelemetry = telemetry[telemetry.length - 1];
	$: currentAstro = getEVA(telemetry[telemetry.length - 1], selectedAstro);

	const suitResources = (eva: Astronaut) => {
		return [
			{
				key: 'Battery Time Left',
				units: ResourceBounds.batt_time_left.units,
				formatter: formatTime,
				range: ResourceBounds.batt_time_left,
				value: eva.batt_time_left,
				color: getColor(eva.batt_time_left, ResourceBounds.batt_time_left)
			},
			{
				key: 'Primary Oxygen Storage',
				units: ResourceBounds.oxy_pri_storage.units,
				formatter: formatDecimals(2),
				range: ResourceBounds.oxy_pri_storage,
				value: eva.oxy_pri_storage,
				color: getColor(eva.oxy_pri_storage, ResourceBounds.oxy_pri_storage)
			},
			{
				key: 'Secondary Oxygen Storage',
				units: ResourceBounds.oxy_sec_storage.units,
				formatter: formatDecimals(2),
				range: ResourceBounds.oxy_sec_storage,
				value: eva.oxy_sec_storage,
				color: getColor(eva.oxy_sec_storage, ResourceBounds.oxy_sec_storage)
			},
			{
				key: 'Primary Oxygen Pressure',
				units: ResourceBounds.oxy_pri_pressure.units,
				formatter: formatDecimals(2),
				range: ResourceBounds.oxy_pri_pressure,
				value: eva.oxy_pri_pressure,
				color: getColor(eva.oxy_pri_pressure, ResourceBounds.oxy_pri_pressure)
			},
			{
				key: 'Secondary Oxygen Pressure',
				units: ResourceBounds.oxy_sec_pressure.units,
				formatter: formatDecimals(2),
				range: ResourceBounds.oxy_sec_pressure,
				value: eva.oxy_sec_pressure,
				color: getColor(eva.oxy_sec_pressure, ResourceBounds.oxy_sec_pressure)
			},
			{
				key: 'Oxygen Time Left',
				units: ResourceBounds.oxy_time_left.units,
				formatter: formatTime,
				range: ResourceBounds.oxy_time_left,
				value: eva.oxy_time_left,
				color: getColor(eva.oxy_time_left, ResourceBounds.oxy_time_left)
			},
			{
				key: 'Coolant Volume',
				units: ResourceBounds.coolant_ml.units,
				formatter: formatDecimals(2),
				range: ResourceBounds.coolant_ml,
				value: eva.coolant_ml,
				color: getColor(eva.coolant_ml, ResourceBounds.oxy_time_left)
			}
		];
	};

	const suitAtmosphere = (eva: Astronaut) => {
		return [
			{
				key: 'Heart Rate',
				units: AtmosphereBounds.heart_rate.units,
				formatter: formatDecimals(2),
				range: AtmosphereBounds.heart_rate,
				value: eva.heart_rate,
				color: getColor(eva.heart_rate, AtmosphereBounds.heart_rate)
			},
			{
				key: 'Oxygen Consumption',
				units: AtmosphereBounds.oxy_consumption.units,
				formatter: formatDecimals(2),
				range: AtmosphereBounds.oxy_consumption,
				value: eva.oxy_consumption,
				color: getColor(eva.oxy_consumption, AtmosphereBounds.oxy_consumption)
			},
			{
				key: 'CO2 Production',
				units: AtmosphereBounds.co2_production.units,
				formatter: formatDecimals(2),
				range: AtmosphereBounds.co2_production,
				value: eva.co2_production,
				color: getColor(eva.co2_production, AtmosphereBounds.co2_production)
			},
			{
				key: 'Suit Pressure Oxygen',
				units: AtmosphereBounds.suit_pressure_oxy.units,
				formatter: formatDecimals(2),
				range: AtmosphereBounds.suit_pressure_oxy,
				value: eva.suit_pressure_oxy,
				color: getColor(eva.suit_pressure_oxy, AtmosphereBounds.suit_pressure_oxy)
			},
			{
				key: 'Suit Pressure CO2',
				units: AtmosphereBounds.suit_pressure_co2.units,
				formatter: formatDecimals(2),
				range: AtmosphereBounds.suit_pressure_co2,
				value: eva.suit_pressure_co2,
				color: getColor(eva.suit_pressure_co2, AtmosphereBounds.suit_pressure_co2)
			},
			{
				key: 'Suit Pressure Other',
				units: AtmosphereBounds.suit_pressure_other.units,
				formatter: formatDecimals(2),
				range: AtmosphereBounds.suit_pressure_other,
				value: eva.suit_pressure_other,
				color: getColor(eva.suit_pressure_other, AtmosphereBounds.suit_pressure_other)
			},
			{
				key: 'Suit Pressure Total',
				units: AtmosphereBounds.suit_pressure_total.units,
				formatter: formatDecimals(2),
				range: AtmosphereBounds.suit_pressure_total,
				value: eva.suit_pressure_total,
				color: getColor(eva.suit_pressure_total, AtmosphereBounds.suit_pressure_total)
			},
			{
				key: 'Helmet Pressure CO2',
				units: AtmosphereBounds.helmet_pressure_co2.units,
				formatter: formatDecimals(2),
				range: AtmosphereBounds.helmet_pressure_co2,
				value: eva.helmet_pressure_co2,
				color: getColor(eva.helmet_pressure_co2, AtmosphereBounds.helmet_pressure_co2)
			}
		];
	};

	const suitHelmet = (eva: Astronaut) => {
		return [
			{
				key: 'Primary Fan Speed',
				units: HelmetBounds.fan_pri_rpm.units,
				formatter: formatDecimals(2),
				range: HelmetBounds.fan_pri_rpm,
				value: eva.fan_pri_rpm,
				color: getColor(eva.fan_pri_rpm, HelmetBounds.fan_pri_rpm)
			},
			{
				key: 'Secondary Fan Speed',
				units: HelmetBounds.fan_sec_rpm.units,
				formatter: formatDecimals(2),
				range: HelmetBounds.fan_sec_rpm,
				value: eva.fan_sec_rpm,
				color: getColor(eva.fan_sec_rpm, HelmetBounds.fan_sec_rpm)
			}
		];
	};

	const suitScrubber = (eva: Astronaut) => {
		return [
			{
				key: 'Scrubber A CO2 Storage',
				units: ScrubberBounds.scrubber_a_co2_storage.units,
				formatter: formatDecimals(2),
				range: ScrubberBounds.scrubber_a_co2_storage,
				value: eva.scrubber_a_co2_storage,
				color: getColor(eva.scrubber_a_co2_storage, ScrubberBounds.scrubber_a_co2_storage)
			},
			{
				key: 'Scrubber B CO2 Storage',
				units: ScrubberBounds.scrubber_b_co2_storage.units,
				formatter: formatDecimals(2),
				range: ScrubberBounds.scrubber_b_co2_storage,
				value: eva.scrubber_b_co2_storage,
				color: getColor(eva.scrubber_b_co2_storage, ScrubberBounds.scrubber_b_co2_storage)
			}
		];
	};

	const suitTemperature = (eva: Astronaut) => {
		return [
			{
				key: 'Temperature',
				units: TemperatureBounds.temperature.units,
				formatter: formatDecimals(2),
				range: TemperatureBounds.temperature,
				value: eva.temperature,
				color: getColor(eva.temperature, TemperatureBounds.temperature)
			},
			{
				key: 'Coolant Gas Pressure',
				units: TemperatureBounds.coolant_gas_pressure.units,
				formatter: formatDecimals(2),
				range: TemperatureBounds.coolant_gas_pressure,
				value: eva.coolant_gas_pressure,
				color: getColor(eva.coolant_gas_pressure, TemperatureBounds.coolant_gas_pressure)
			},
			{
				key: 'Coolant Liquid Pressure',
				units: TemperatureBounds.coolant_liquid_pressure.units,
				formatter: formatDecimals(2),
				range: TemperatureBounds.coolant_liquid_pressure,
				value: eva.coolant_liquid_pressure,
				color: getColor(eva.coolant_liquid_pressure, TemperatureBounds.coolant_liquid_pressure)
			}
		];
	};

	$: params = [
		...suitResources(currentAstro),
		...suitAtmosphere(currentAstro),
		...suitHelmet(currentAstro),
		...suitScrubber(currentAstro),
		...suitTemperature(currentAstro)
	];

	let currentNotifs = [];
	let armed = [];

	function sendNotification(param) {
		if (getColor(param.value, param.range) != goodColor) {
			if (armed.includes(param.key) && !currentNotifs.includes(param.key)) {
				console.log('Somthing is out of bounds');
				const url = 'http://localhost:8181/notifications/';

				const data = {
					content: `${param.key} is out of range.`,
					severity: 0
				};

				const options = {
					method: 'POST',
					headers: {
						accept: 'application/json',
						'Content-Type': 'application/json'
					},
					body: JSON.stringify(data)
				};

				fetch(url, options)
					.then((response) => {
						console.log('Response Status Code:', response.status);
						return response.json();
					})
					.then((data) => {
						console.log('Response Body:', data);
					})
					.catch((error) => {
						console.error('Error:', error);
					});
				armed.splice(armed.indexOf(param.key), 1);
				currentNotifs.push(param.key);
			}
		}
	}

	function arm(param) {
		if (getColor(param.value, param.range) == goodColor) {
			if (!armed.includes(param.key)) {
				armed.push(param.key);
				// conditionally remove from sent notifications
				let index = currentNotifs.indexOf(param.key);
				if (index > -1) {
					currentNotifs.splice(index, 1);
				}
			}
		}
	}

	$: _ = params.forEach((a) => sendNotification(a));
	$: __ = params.forEach((a) => arm(a));

	const categories = (eva: Astronaut) => {
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
										<p class={'text-sm font-medium ' + item['color']}>
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
