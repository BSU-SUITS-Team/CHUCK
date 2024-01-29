<script>
	/** @type {import('./$types').PageData} */
	export let data;
	import {
		TableSearch,
		TableHead,
		TableHeadCell,
		TableBody,
		TableBodyRow,
		TableBodyCell,
		Card,
		Heading
	} from 'flowbite-svelte';
	import Chart from './chart.svelte';
	import {
		CheckCircleOutline,
		ExclamationCircleOutline,
		XCircleOutline
	} from 'flowbite-svelte-icons';

	let astronauts = [
		{
			name: 'John Doe',
			oxygen: 99,
			battery: 22,
			co2: 178,
			heartRate: 0,
			temperature: 99,
			location: 'Mars',
			fanSpeed: 68
		},
		{
			name: 'Jane Doe',
			oxygen: 70,
			battery: 21,
			co2: 178,
			heartRate: 100,
			temperature: 4,
			location: 'Mars',
			fanSpeed: 94
		},		
		{
			name: 'Bob Doe',
			oxygen: 90,
			battery: 21,
			co2: 178,
			heartRate: 100,
			temperature: 98,
			location: 'Mars',
			fanSpeed: 94
		},
		{
			name: 'Other',
			oxygen: 90,
			battery: 21,
			co2: 178,
			heartRate: 100,
			temperature: 98,
			location: 'Mars',
			fanSpeed: 94
		}
	];

	// total range, warning range, nominal range
	let ranges = {
		oxygen: [
			[0, 100],
			[25, 100],
			[75, 100]
		],
		battery: [
			[5, 24],
			[10, 24],
			[20, 24]
		],
		co2: [
			[0, 2000],
			[0, 1000],
			[50, 500]
		],
		heartRate: [
			[30, 200],
			[50, 160],
			[60, 130]
		],
		temperature: [
			[90, 110],
			[95, 100],
			[97, 99]
		]
	};

	let units = {
		oxygen: '%',
		battery: 'V',
		co2: 'ppm',
		heartRate: 'bpm',
		temperature: '°F'
	};

	const errorColor = 'text-red-500 font-bold';
	const warningColor = 'text-orange-500 font-bold';
	const goodColor = 'text-green-500';

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
		if (battery < 10) {
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

	function Validate(name, value) {
		if (name in ranges) {
			let range = ranges[name];
			// if in nominal range
			if (value >= range[2][0] && value <= range[2][1]) {
				return goodColor;
			} else if (value >= range[1][0] && value <= range[1][1]) {
				return warningColor;
			} else {
				return errorColor;
			}
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

		return worst;
	}
</script>

<div class=" mr-24 overflow-auto flex flex-row flex-wrap">
	{#each astronauts as astro}
		<Card class="m-2 grow">
			<div class="flex justify-between flex-row border-b mb-2 pb-2">
				<div>
					<h5 class="leading-none text-3xl font-bold text-gray-900 dark:text-white pb-2">
						{astro.name}
					</h5>
					<p class="text-base font-normal text-gray-500 dark:text-gray-400">Biometrics</p>
				</div>
				<div
					class="flex items-top px-2.5 py-0.5 text-base font-semibold text-red-500 dark:text-red-500 text-center"
				>
					{#if GetWorst(astro) == errorColor}
						<ExclamationCircleOutline class="w-5 h-5 ml-1 mt-2 text-orange-500" />
					{:else}
						<CheckCircleOutline class="w-5 h-5 ml-1 mt-2 text-green-500" />
					{/if}
				</div>
			</div>
			{#each Object.keys(astro) as key}
				{#if key != 'name' && key != 'location' && key != 'fanSpeed'}
					<div class="flex flex-row justify-between pb-2">
						<p class=" text-black dark:text-gray-300">
							{key}
							<span class={Validate(key, astro[key])}> {astro[key]}{units[key]}</span>
						</p>
						<Chart
							fullRange={ranges[key][0]}
							warnRange={ranges[key][1]}
							nominalRange={ranges[key][2]}
							value={astro[key]}
						/>
					</div>
				{/if}
			{/each}
		</Card>
	{/each}
</div>
