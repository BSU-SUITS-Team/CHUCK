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
		type TelemetryEvent,
		getAstronauts,
		getEVA,
		compareValueToBounds,
		ResourceBounds,
		HelmetBounds,
		AtmosphereBounds,
		ScrubberBounds,
		TemperatureBounds,
		Threshold,

		type Astronaut

	} from '$lib/biometrics';

	let selected = undefined;

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

	$: telemetry = sampleTelemetry;

	const suitResources = (eva: Astronaut) => {
		return [
			{
				key: "Battery Time Left",
				units: ResourceBounds.batt_time_left.units,
				value: eva.batt_time_left,
				color: getColor(eva.batt_time_left, ResourceBounds.batt_time_left),
			},
			{
				key: "Primary Oxygen Storage",
				units: ResourceBounds.oxy_pri_storage.units,
				value: eva.oxy_pri_storage,
				color: getColor(eva.oxy_pri_storage, ResourceBounds.oxy_pri_storage),
			},
			{
				key: "Secondary Oxygen Storage",
				units: ResourceBounds.oxy_sec_storage.units,
				value: eva.oxy_sec_storage,
				color: getColor(eva.oxy_sec_storage, ResourceBounds.oxy_sec_storage),
			},
			{
				key: "Primary Oxygen Pressure",
				units: ResourceBounds.oxy_pri_pressure.units,
				value: eva.oxy_pri_pressure,
				color: getColor(eva.oxy_pri_pressure, ResourceBounds.oxy_pri_pressure),
			},
			{
				key: "Secondary Oxygen Pressure",
				units: ResourceBounds.oxy_sec_pressure.units,
				value: eva.oxy_sec_pressure,
				color: getColor(eva.oxy_sec_pressure, ResourceBounds.oxy_sec_pressure),
			},
			{
				key: "Oxygen Time Left",
				units: ResourceBounds.oxy_time_left.units,
				value: eva.oxy_time_left,
				color: getColor(eva.oxy_time_left, ResourceBounds.oxy_time_left),
			},
			{
				key: "Coolant Volume",
				units: ResourceBounds.oxy_time_left.units,
				value: eva.oxy_time_left,
				color: getColor(eva.oxy_time_left, ResourceBounds.oxy_time_left),
			},
		];
	};

	const suitAtmosphere = (eva: Astronaut) => {
		return [
			{
				key: "Heart Rate",
				units: AtmosphereBounds.heart_rate.units,
				value: eva.heart_rate,
				color: getColor(eva.heart_rate, AtmosphereBounds.heart_rate),
			},
			{
				key: "Oxygen Consumption",
				units: AtmosphereBounds.oxy_consumption.units,
				value: eva.oxy_consumption,
				color: getColor(eva.oxy_consumption, AtmosphereBounds.oxy_consumption),
			},
			{
				key: "CO2 Production",
				units: AtmosphereBounds.co2_production.units,
				value: eva.co2_production,
				color: getColor(eva.co2_production, AtmosphereBounds.co2_production),
			},
			{
				key: "Suit Pressure Oxygen",
				units: AtmosphereBounds.suit_pressure_oxy.units,
				value: eva.suit_pressure_oxy,
				color: getColor(eva.suit_pressure_oxy, AtmosphereBounds.suit_pressure_oxy),
			},
			{
				key: "Suit Pressure CO2",
				units: AtmosphereBounds.suit_pressure_co2.units,
				value: eva.suit_pressure_co2,
				color: getColor(eva.suit_pressure_co2, AtmosphereBounds.suit_pressure_co2),
			},
			{
				key: "Suit Pressure Other",
				units: AtmosphereBounds.suit_pressure_other.units,
				value: eva.suit_pressure_other,
				color: getColor(eva.suit_pressure_other, AtmosphereBounds.suit_pressure_other),
			},
			{
				key: "Suit Pressure Total",
				units: AtmosphereBounds.suit_pressure_total.units,
				value: eva.suit_pressure_total,
				color: getColor(eva.suit_pressure_total, AtmosphereBounds.suit_pressure_total),
			},
			{
				key: "Helmet Pressure CO2",
				units: AtmosphereBounds.helmet_pressure_co2.units,
				value: eva.helmet_pressure_co2,
				color: getColor(eva.helmet_pressure_co2, AtmosphereBounds.helmet_pressure_co2),
			},
		];
	};

	const suitHelmet = (eva: Astronaut) => {
		return [
			{
				key: "Primary Fan Speed",
				units: HelmetBounds.fan_pri_rpm.units,
				value: eva.fan_pri_rpm,
				color: getColor(eva.fan_pri_rpm, HelmetBounds.fan_pri_rpm),
			},
			{
				key: "Secondary Fan Speed",
				units: HelmetBounds.fan_sec_rpm.units,
				value: eva.fan_sec_rpm,
				color: getColor(eva.fan_sec_rpm, HelmetBounds.fan_sec_rpm),
			},
		]
	};

	const suitScrubber = (eva: Astronaut) => {
		return [
			{
				key: "Scrubber A CO2 Storage",
				units: ScrubberBounds.scrubber_a_co2_storage.units,
				value: eva.scrubber_a_co2_storage,
				color: getColor(eva.scrubber_a_co2_storage, ScrubberBounds.scrubber_a_co2_storage),
			},
			{
				key: "Scrubber B CO2 Storage",
				units: ScrubberBounds.scrubber_b_co2_storage.units,
				value: eva.scrubber_b_co2_storage,
				color: getColor(eva.scrubber_b_co2_storage, ScrubberBounds.scrubber_b_co2_storage),
			},
		]
	};

	const suitTemperature = (eva: Astronaut) => {
		return [
			{
				key: "Temperature",
				units: TemperatureBounds.temperature.units,
				value: eva.temperature,
				color: getColor(eva.temperature, TemperatureBounds.temperature),
			},
			{
				key: "Coolant Gas Pressure",
				units: TemperatureBounds.coolant_gas_pressure.units,
				value: eva.coolant_gas_pressure,
				color: getColor(eva.coolant_gas_pressure, TemperatureBounds.coolant_gas_pressure),
			},
			{
				key: "Coolant Liquid Pressure",
				units: TemperatureBounds.coolant_liquid_pressure.units,
				value: eva.coolant_liquid_pressure,
				color: getColor(eva.coolant_liquid_pressure, TemperatureBounds.coolant_liquid_pressure),
			},
		]
	};

	const categories = (eva: Astronaut) => {
		return {
			"Suit Resources": suitResources(eva),
			"Suit Atmosphere": suitAtmosphere(eva),
			"Suit Helmet Fan": suitHelmet(eva),
			"Suit CO2 Scrubbers": suitScrubber(eva),
			"Suit Temperature": suitTemperature(eva),
		}
	}
</script>

<div class="h-full mr-24 overflow-auto pt-8">
	<Tabs>
		{#each getAstronauts(telemetry) as astro}
			<TabItem open title={astro} on:click={() => {selected = astro}}>
				<div class="flex gap-2 flex-wrap">
					{#each Object.keys(categories(getEVA(telemetry, astro))) as category}						
						<Card>
							<div class="flex justify-between items-center mb-4">
								<h5 class="text-xl font-bold leading-none text-gray-900 dark:text-white">
									{category}
								</h5>
							</div>
							<Listgroup items={categories(getEVA(telemetry, astro))[category]} let:item class="border-0 dark:!bg-transparent">
								<div class="flex items-center space-x-4 rtl:space-x-reverse">
									<div class="flex-1 min-w-0">
										<p class="text-sm font-medium text-gray-900 dark:text-white">
											{item["key"]}
										</p>
									</div>
									<div class="flex-1 min-w-0">
										<p class="text-sm font-medium text-gray-900 dark:text-white">
											{item["units"]}
										</p>
									</div>
									<div class="flex-1 min-w-0">
										<p class={"text-sm font-medium "+item["color"]}>
											{item["value"]}
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
</div>
