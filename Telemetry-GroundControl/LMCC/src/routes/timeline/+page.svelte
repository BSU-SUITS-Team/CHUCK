<script>
	let titleClass = 'text-center border-b border-r text-2xl p-2';

	let timeline = {
		col1: [
			{
				name: 'test',
				duration: 15,
				elements: [
					{ description: 'first', duration: 5 },
					{ description: 'second', duration: 10 }
				]
			},
			{
				name: 'workout',
				duration: 60,
				elements: [
					{ description: 'warm-up', duration: 10 },
					{ description: 'cardio', duration: 20 },
					{ description: 'strength training', duration: 25 },
					{ description: 'cool-down', duration: 5 }
				]
			},
			{
				name: 'meeting',
				duration: 90,
				elements: [
					{ description: 'introduction', duration: 10 },
					{ description: 'presentation', duration: 50 },
					{ description: 'Q&A', duration: 20 },
					{ description: 'wrap-up', duration: 10 }
				]
			},
			{
				name: 'cleaning',
				duration: 15,
				elements: [{ description: 'dust surfaces', duration: 15 }]
			},
			{
				name: 'cleaning',
				duration: 45,
				elements: [
					{ description: 'dust surfaces', duration: 10 },
					{ description: 'vacuum', duration: 20 },
					{ description: 'mop floors', duration: 15 }
				]
			},
			{
				name: 'workout',
				duration: 60,
				elements: [
					{ description: 'warm-up', duration: 10 },
					{ description: 'cardio', duration: 20 },
					{ description: 'strength training', duration: 25 },
					{ description: 'cool-down', duration: 5 }
				]
			}
		]
	};

	timeline.col2 = [...timeline.col1];
	timeline.col2.sort(() => Math.random() - 0.5);
	timeline.col3 = [...timeline.col1];
	timeline.col3.sort(() => Math.random() - 0.5);
	timeline.col4 = [...timeline.col1];
	timeline.col4.sort(() => Math.random() - 0.5);

	function startPos(list, i) {
		let elapsed = 0;
		for (let j = 0; j < i; j++) {
			elapsed += Math.floor(list[j]['duration'] / 5);
		}
		return elapsed + 2;
	}

	function endPos(list, i) {
		let start = startPos(list, i);
		return start + Math.floor(list[i]['duration'] / 5);
	}
</script>

<div class="w-full h-full ml-20 pr-64 mt-10 mb-10">
	<div class="grid rounded-xl border h-full timeline-grid items-stretch justify-items-stretch">
		<div class="{titleClass}  border-b">TIME</div>
		<div class={titleClass}>LMCC 1</div>
		<div class={titleClass}>LMCC 2</div>
		<div class={titleClass}>EV 1</div>
		<div class={titleClass}>EV 2</div>

		<!-- Column 1	 -->
		{#each timeline.col1 as col1, i}
			<div
				class="bg-gray-50 col-start-2 m-1 p-2"
				style="grid-row-start: {startPos(timeline.col1, i)};
				grid-row-end: {endPos(timeline.col1, i)};"
			>
				<h1 class="text-2xl">{col1.name}</h1>
				<ol>
					{#each col1.elements as e}
						<li class="text-sm">
							{e.description}
						</li>
					{/each}
				</ol>
			</div>
		{/each}

		<!-- Column 2	 -->
		{#each timeline.col2 as col2, i}
			<div
				class="bg-gray-50 col-start-3 m-1 p-2"
				style="grid-row-start: {startPos(timeline.col2, i)};
				grid-row-end: {endPos(timeline.col2, i)};"
			>
				<h1 class="text-2xl">{col2.name}</h1>
				<ol>
					{#each col2.elements as e}
						<li class="text-sm">
							{e.description}
						</li>
					{/each}
				</ol>
			</div>
		{/each}

		<!-- Column 3	 -->
		{#each timeline.col3 as col3, i}
			<div
				class="bg-gray-50 col-start-4 m-1 p-2"
				style="grid-row-start: {startPos(timeline.col3, i)};
				grid-row-end: {endPos(timeline.col3, i)};"
			>
				<h1 class="text-2xl">{col3.name}</h1>
				<ol>
					{#each col3.elements as e}
						<li class="text-sm">
							{e.description}
						</li>
					{/each}
				</ol>
			</div>
		{/each}

		<!-- Column 3	 -->
		{#each timeline.col4 as col4, i}
			<div
				class="bg-gray-50 col-start-5 m-1 p-2"
				style="grid-row-start: {startPos(timeline.col4, i)};
				grid-row-end: {endPos(timeline.col4, i)};"
			>
				<h1 class="text-2xl">{col4.name}</h1>
				<ol>
					{#each col4.elements as e}
						<li class="text-sm">
							{e.description}
						</li>
					{/each}
				</ol>
			</div>
		{/each}
	</div>
</div>

<style>
	.timeline-grid {
		grid-template-columns: 5rem 1fr 1fr 1fr 1fr;
		grid-template-rows: 3rem;
	}
</style>
