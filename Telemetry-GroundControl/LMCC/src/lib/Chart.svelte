<script lang="ts">
	export let options: Record<string, unknown> = {};

	let className = '';
	export { className as class };

	function initChart(node: HTMLDivElement, chartOptions: Record<string, unknown>) {
		let chart:
			| {
					render: () => Promise<void>;
					updateOptions: (options: Record<string, unknown>) => void;
					destroy: () => void;
			  }
			| undefined;
		let destroyed = false;

		async function renderChart() {
			const ApexCharts = (await import('apexcharts')).default;
			if (destroyed) return;

			chart = new ApexCharts(node, chartOptions);
			await chart.render();
		}

		renderChart();

		return {
			update(nextOptions: Record<string, unknown>) {
				chartOptions = nextOptions;
				chart?.updateOptions(chartOptions);
			},
			destroy() {
				destroyed = true;
				chart?.destroy();
			}
		};
	}
</script>

<div use:initChart={options} class={className}></div>
