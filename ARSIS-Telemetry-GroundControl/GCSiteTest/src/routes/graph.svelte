<script>
	import { Chart, Card, A, Button, Dropdown, DropdownItem } from 'flowbite-svelte';
	import { ChevronRightSolid, ChevronDownSolid, ChevronUpSolid } from 'flowbite-svelte-icons';

	export let name = 'Power Level';
	export let status = '12.1V';
	export let lastDelta = '12%';
	export let graphdata = [0, 10, 20, 30, 40, 50, 60, 70, 80, 90];

	let options = {
		chart: {
			height: '100%',
			maxWidth: '100%',
			type: 'area',
			fontFamily: 'Inter, sans-serif',
			dropShadow: {
				enabled: false
			},
			toolbar: {
				show: false
			}
		},
		tooltip: {
			enabled: false,
			x: {
				show: false
			}
		},
		fill: {
			type: 'gradient',
			gradient: {
				opacityFrom: 0.55,
				opacityTo: 0,
				shade: '#1C64F2',
				gradientToColors: ['#1C64F2']
			}
		},
		dataLabels: {
			enabled: false
		},
		stroke: {
			width: 6
		},
		grid: {
			show: true,
			strokeDashArray: 4,
			padding: {
				left: 2,
				right: 2,
				top: 0
			}
		},
		series: [
			{
				name: 'Battery 1',
				data: graphdata,
				color: '#1A56DB'
			}
		],
		xaxis: {
			labels: {
				show: false
			},
			axisBorder: {
				show: false
			},
			axisTicks: {
				show: false
			}
		},
		yaxis: {
			show: false
		}
	};
</script>

<Card>
	<div class="flex justify-between">
		<div>
			<h5 class="leading-none text-3xl font-bold text-gray-900 dark:text-white pb-2">{status}</h5>
			<p class="text-base font-normal text-gray-500 dark:text-gray-400">{name}</p>
		</div>
		<div
			class="flex items-center px-2.5 py-0.5 text-base font-semibold text-red-500 dark:text-red-500 text-center"
		>
			{lastDelta}
			{#if lastDelta.includes('-')}
				<ChevronDownSolid class="w-3 h-3 ml-1" />
			{:else}
				<ChevronUpSolid class="w-3 h-3 ml-1" />
			{/if}
		</div>
	</div>
	<Chart {options} />
	<div
		class="grid grid-cols-1 items-center border-gray-200 border-t dark:border-gray-700 justify-between"
	>
		<div class="flex justify-between items-center pt-5">
			<Button
				class="text-sm font-medium text-gray-500 dark:text-gray-400 hover:text-gray-900 text-center inline-flex items-center dark:hover:text-white bg-transparent hover:bg-transparent dark:bg-transparent dark:hover:bg-transparent focus:ring-transparent dark:focus:ring-transparent py-0"
				>Last Hour<ChevronDownSolid class="w-2.5 m-2.5 ml-1.5" /></Button
			>
			<Dropdown class="w-40" offset="-6">
				<DropdownItem>Last 5 Minuites</DropdownItem>
				<DropdownItem>Last 15 Minuites</DropdownItem>
				<DropdownItem>Last Hour</DropdownItem>
			</Dropdown>
			<A
				href="/rover"
				class="uppercase text-sm font-semibold hover:text-primary-700 dark:hover:text-primary-500 rounded-lg hover:bg-gray-100 dark:hover:bg-gray-700 dark:focus:ring-gray-700 dark:border-gray-700 px-3 py-2 hover:no-underline"
			>
				Details
				<ChevronRightSolid class="w-2.5 h-2.5 ml-1.5" />
			</A>
		</div>
	</div>
</Card>
