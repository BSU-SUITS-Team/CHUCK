<script lang="ts">
	import {
		TableHead,
		TableHeadCell,
		TableBody,
		TableBodyRow,
		TableBodyCell,
		Table
	} from 'flowbite-svelte';
	import { biometrics } from '$lib/biometrics.js';

	const errorColor = 'p-2 text-align-left rounded-md text-red-500';
	const warningColor = 'p-2 text-align-left rounded-md text-orange-400';
	const goodColor = 'p-2 text-align-left rounded-md ';

	function TemperatureValidation(temperature) {
		let delta = Math.abs(temperature - 98.6);
		if (delta > 3) {
			return errorColor;
		} else if (delta > 1) {
			return warningColor;
		} else {
			return goodColor;
		}
	}

	function HeartRateValidation(heartRate) {
		let target = 90;
		let delta = Math.abs(heartRate - target);
		if (delta > 40) {
			return errorColor;
		} else if (delta > 20) {
			return warningColor;
		} else {
			return goodColor;
		}
	}

	function CO2Validation(co2) {
		co2 = Math.abs(co2);
		if (co2 > 1000) {
			return errorColor;
		} else if (co2 > 500) {
			return warningColor;
		} else {
			return goodColor;
		}
	}

	function BatteryValidation(battery) {
		if (battery < 20) {
			return errorColor;
		} else if (battery < 50) {
			return warningColor;
		} else {
			return goodColor;
		}
	}

	function OxygenValidation(oxygen) {
		if (oxygen < 25) {
			return errorColor;
		} else if (oxygen < 50) {
			return warningColor;
		} else {
			return goodColor;
		}
	}

	//This code is appaling, but it works
	function GetWorst(astonaut) {
		let worst = goodColor;
		if (OxygenValidation(astonaut.oxygen) == errorColor) {
			worst = errorColor;
		} else if (OxygenValidation(astonaut.oxygen) == warningColor && worst != errorColor) {
			worst = warningColor;
		}
		if (BatteryValidation(astonaut.battery) == errorColor) {
			worst = errorColor;
		} else if (BatteryValidation(astonaut.battery) == warningColor && worst != errorColor) {
			worst = warningColor;
		}
		if (CO2Validation(astonaut.co2) == errorColor) {
			worst = errorColor;
		} else if (CO2Validation(astonaut.co2) == warningColor && worst != errorColor) {
			worst = warningColor;
		}
		if (HeartRateValidation(astonaut.heartRate) == errorColor) {
			worst = errorColor;
		} else if (HeartRateValidation(astonaut.heartRate) == warningColor && worst != errorColor) {
			worst = warningColor;
		}
		if (TemperatureValidation(astonaut.temperature) == errorColor) {
			worst = errorColor;
		} else if (TemperatureValidation(astonaut.temperature) == warningColor && worst != errorColor) {
			worst = warningColor;
		}

		// const bgError = 'bg-red-400';
		// const bgWarning = 'bg-orange-400';

		// if (worst == errorColor){
		// 	return bgError;
		// }
		// else if (worst == warningColor){
		// 	return bgWarning;
		// }
		// else{
		// 	return '';
		// }
		return worst;
	}
</script>

<div class="h-full mr-24 overflow-auto pt-8">
	<Table hoverable>
		<TableHead>
			<TableHeadCell>EVA</TableHeadCell>
			<TableHeadCell>Heart Rate (BPM)</TableHeadCell>
			<TableHeadCell>Temperature (&deg;F)</TableHeadCell>
			<TableHeadCell>Primary O2 Storage (%)</TableHeadCell>
			<TableHeadCell>Primary O2 Pressure (PSI)</TableHeadCell>
			<TableHeadCell>Secondary O2 Storage (%)</TableHeadCell>
			<TableHeadCell>Secondary O2 Pressure (PSI)</TableHeadCell>
			<TableHeadCell>O2 Consumption (PSI/min)</TableHeadCell>
			<TableHeadCell>CO2 Production (PSI/min)</TableHeadCell>
			<TableHeadCell>O2 Time Left (hh:mm:ss)</TableHeadCell>
			<TableHeadCell>Suit O2 Pressure (PSI)</TableHeadCell>
			<TableHeadCell>Suit CO2 Pressure (PSI)</TableHeadCell>
			<TableHeadCell>Suit Other Pressure (PSI)</TableHeadCell>
			<TableHeadCell>Suit Total Pressure (PSI)</TableHeadCell>
			<TableHeadCell>Helmet CO2 Pressure (PSI)</TableHeadCell>
			<TableHeadCell>Scrubber A Pressure (PSI)</TableHeadCell>
			<TableHeadCell>Scrubber B Pressure (PSI)</TableHeadCell>
			<TableHeadCell>Primary Fan (RPM)</TableHeadCell>
			<TableHeadCell>Secondary Fan (RPM)</TableHeadCell>
			<TableHeadCell>Coolant Gas Pressure (PSI)</TableHeadCell>
			<TableHeadCell>Coolant Liquid Pressure (PSI)</TableHeadCell>
			<TableHeadCell>Coolant Volume (mL)</TableHeadCell>
		</TableHead>
		<TableBody>
			{#each $biometrics as astro}
				<TableBodyRow>
					<TableBodyCell>
						<span>
							{astro.eva}
						</span>
					</TableBodyCell>
					<TableBodyCell>
						<span class={HeartRateValidation(astro.heart_rate)}>
							{astro.heart_rate.toLocaleString()}
						</span>
					</TableBodyCell>
					<TableBodyCell>
						<span class={TemperatureValidation(astro.temperature)}>
							{astro.temperature.toLocaleString()}
						</span>
					</TableBodyCell>
					<TableBodyCell>
						<span>
							{astro.oxy_pri_storage}
						</span>
					</TableBodyCell>
					<TableBodyCell>
						<span>
							{astro.oxy_pri_pressure}
						</span>
					</TableBodyCell>
					<TableBodyCell>
						<span>
							{astro.oxy_sec_storage}
						</span>
					</TableBodyCell>
					<TableBodyCell>
						<span>
							{astro.oxy_sec_pressure}
						</span>
					</TableBodyCell>
					<TableBodyCell>
						<span>
							{astro.oxy_consumption}
						</span>
					</TableBodyCell>
					<TableBodyCell>
						<span>
							{astro.co2_production}
						</span>
					</TableBodyCell>
					<TableBodyCell>
						<span>
							{astro.oxy_time_left}
						</span>
					</TableBodyCell>
					<TableBodyCell>
						<span>
							{astro.suit_pressure_oxy}
						</span>
					</TableBodyCell>
					<TableBodyCell>
						<span>
							{astro.suit_pressure_co2}
						</span>
					</TableBodyCell>
					<TableBodyCell>
						<span>
							{astro.suit_pressure_other}
						</span>
					</TableBodyCell>
					<TableBodyCell>
						<span>
							{astro.suit_pressure_total}
						</span>
					</TableBodyCell>
					<TableBodyCell>
						<span>
							{astro.helmet_pressure_co2}
						</span>
					</TableBodyCell>
					<TableBodyCell>
						<span>
							{astro.scrubber_a_co2_storage}
						</span>
					</TableBodyCell>
					<TableBodyCell>
						<span>
							{astro.scrubber_b_co2_storage}
						</span>
					</TableBodyCell>
					<TableBodyCell>
						<span>
							{astro.fan_pri_rpm}
						</span>
					</TableBodyCell>
					<TableBodyCell>
						<span>
							{astro.fan_sec_rpm}
						</span>
					</TableBodyCell>
					<TableBodyCell>
						<span>
							{astro.coolant_gas_pressure}
						</span>
					</TableBodyCell>
					<TableBodyCell>
						<span>
							{astro.coolant_liquid_pressure}
						</span>
					</TableBodyCell>
					<TableBodyCell>
						<span>
							{astro.coolant_ml}
						</span>
					</TableBodyCell>
					<!-- <TableBodyCell>
						<span class={OxygenValidation(astro.oxygen)}>
							{astro.oxygen.toLocaleString()}%
						</span>
					</TableBodyCell> -->
					<!-- <TableBodyCell>
						<span class={BatteryValidation(astro.battery)}>
							{astro.battery.toLocaleString()}%
						</span>
					</TableBodyCell> -->
					<!-- <TableBodyCell>
						<span class={CO2Validation(astro.co2)}>{astro.co2.toLocaleString()} PPM</span>
					</TableBodyCell> -->
				</TableBodyRow>
			{/each}
		</TableBody>
	</Table>
</div>
