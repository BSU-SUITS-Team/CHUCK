<script>
	import {
		TableHead,
		TableHeadCell,
		TableBody,
		TableBodyRow,
		TableBodyCell,
		Table
	} from 'flowbite-svelte';

	// let astronauts = [
	// 	{
	// 		name: 'EV-1',
	// 		oxygen: 99,
	// 		battery: 14,
	// 		co2: 178,
	// 		heartRate: 90,
	// 		temperature: 98,
	// 		location: 'Mars',
	// 		fanSpeed: 68
	// 	},
	// 	{
	// 		name: 'EV-2',
	// 		oxygen: 70,
	// 		battery: 100,
	// 		co2: 178,
	// 		heartRate: 100,
	// 		temperature: 100,
	// 		location: 'Mars',
	// 		fanSpeed: 94
	// 	}
	// ];

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
			<TableHeadCell>Heart Rate</TableHeadCell>
			<TableHeadCell>Temperature</TableHeadCell>
			<TableHeadCell>Primary O2 Storage</TableHeadCell>
			<TableHeadCell>Primary O2 Pressure</TableHeadCell>
			<TableHeadCell>Secondary O2 Storage</TableHeadCell>
			<TableHeadCell>Secondary O2 Pressure</TableHeadCell>
			<TableHeadCell>Suit O2 Pressure</TableHeadCell>
			<TableHeadCell>Suit CO2 Pressure</TableHeadCell>
			<TableHeadCell>Suit Other Pressure</TableHeadCell>
			<TableHeadCell>Suit Total Pressure</TableHeadCell>
			<TableHeadCell>Helmet CO2 Pressure</TableHeadCell>
			<TableHeadCell>Scrubber A Pressure</TableHeadCell>
			<TableHeadCell>Scrubber B Pressure</TableHeadCell>
			<TableHeadCell>Primary Fan</TableHeadCell>
			<TableHeadCell>Secondary Fan</TableHeadCell>
			<TableHeadCell>H2O Gas Pressure</TableHeadCell>
			<TableHeadCell>H2O Liquid Pressure</TableHeadCell>
			<TableHeadCell>CO2 Production</TableHeadCell>
			<TableHeadCell>O2 Time Left</TableHeadCell>
			<TableHeadCell>O2 Consumption</TableHeadCell>
			<TableHeadCell>Coolant</TableHeadCell>
		</TableHead>
		<TableBody class="divide-y">
			{#each astronauts as astro}
				<TableBodyRow>
					<TableBodyCell>
						<span>
							{astro.name}
						</span>
					</TableBodyCell>
					<TableBodyCell>
						<span class={OxygenValidation(astro.oxygen)}>
							{astro.oxygen.toLocaleString()}%
						</span>
					</TableBodyCell>
					<TableBodyCell>
						<span class={BatteryValidation(astro.battery)}>
							{astro.battery.toLocaleString()}%
						</span>
					</TableBodyCell>
					<TableBodyCell>
						<span class={CO2Validation(astro.co2)}>{astro.co2.toLocaleString()} PPM</span>
					</TableBodyCell>
					<TableBodyCell>
						<span class={HeartRateValidation(astro.heartRate)}>
							{astro.heartRate.toLocaleString()} BPM
						</span>
					</TableBodyCell>
					<TableBodyCell>
						<span class={TemperatureValidation(astro.temperature)}>
							{astro.temperature.toLocaleString()} &deg;F
						</span>
					</TableBodyCell>
				</TableBodyRow>
			{/each}
		</TableBody>
	</Table>
</div>
