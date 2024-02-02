<script>
	import { Chart, Card, A, Button, Dropdown, DropdownItem } from 'flowbite-svelte';
	import { ChevronRightSolid, ChevronDownSolid, ChevronUpSolid } from 'flowbite-svelte-icons';

	export let name = 'Power Level';
	export let status = '12.1V';
	export let graphdata = [0, 10, 20, 30, 40, 50, 60, 70, 80, 90];

	let options = {
		chart: {
			height: '90%',
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
			strokeDashArray: 8,
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
		<p class="text-base font-normal text-gray-500 dark:text-gray-400">{name}</p>
		<h5 class="leading-none text-2xl font-bold text-gray-900 dark:text-white">{status}</h5>
	</div>
	<Chart {options} />
</Card>
