<script>
	import { Heading, Span, Button, Spinner } from 'flowbite-svelte';
	import Graph from '../graph.svelte';
	import { keepables, graphdata } from '../store';
	import EmptyVideo from '../EmptyVideo.svelte';

	let data = [
		['Battery 1:', '72%'],
		['Battery 2:', '76%'],
		['Location:', '(31.9686, 99.9018)'],
		['Status:', 'Active']
	];

	const graphs = [
		{
			graphdata: [14.1, 13.9, 13.8, 13.7, 12.7, 12.1],
			lastDelta: '-12.3%',
			name: 'Battery 1',
			status: '12.1V'
		},
		{
			graphdata: [14.0, 13.8, 13.8, 13.4, 13.3, 12.3],
			lastDelta: '-10.7%',
			name: 'Battery 2',
			status: '12.3V'
		}
	];
</script>

<div class="h-fit m-4">
	<br />
	<div class="border-b p-2 dark:border-gray-700">
		<Heading tag="h1">ROVER</Heading>
	</div>
	<br />
	<Heading tag="h3">Stats</Heading>
	<br />
	<!-- <Button color="alternative">Battery 1: &nbsp <Span highlight>72%</Span></Button>
    <Button color="alternative">Battery 2: &nbsp <Span highlight>76%</Span></Button>
    <Button color="alternative">Location: &nbsp (<Span highlight>31.9686</Span>,<Span highlight>99.9018</Span>)</Button> -->
	<div style="width: 40rem;">
		{#each data as item}
			<Button color="alternative" on:click={() => keepables.addElement('ROVER', item)} class="m-1">
				{item[0]} &nbsp
				<Span highlight>{item[1]}</Span>
			</Button>
		{/each}
	</div>
	<br />
	<br />
	<Heading tag="h3">Actions</Heading>
	<br />
	<Button color="dark">Collect Sample</Button>
	<Button color="dark">Drop Sample</Button>
	<Button color="dark">Take Picture</Button>
	<Button color="dark">Return</Button>
	<Button>Self Destruct</Button>
	<br />
	<br />
	<Heading tag="h3">Cameras</Heading>
	<br />
	<div class="flex flex-row">
		<div class="p-2">
			<EmptyVideo name="Main Camera"/>
		</div>
		<div class="p-2">
			<EmptyVideo name="Realsense Camera "/>
		</div>
	</div>
	<br />
	<br />
	<Heading tag="h3">Details</Heading>
	<br />
	<div class="flex flex-row flex-wrap">
		{#each graphs as graph}
			<button class="w-96 p-2" on:click={() => graphdata.addGraph("ROVER", graph.name, graph.graphdata)}>
				<Graph {...graph}/>
			</button>
		{/each}
	</div>
	
</div>
