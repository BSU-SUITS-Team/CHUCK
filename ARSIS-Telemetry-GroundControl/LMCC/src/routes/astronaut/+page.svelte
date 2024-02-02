<script>
	import {
		Heading,
		Span,
		Button,
		Card,
		Table,
		TableBody,
		TableBodyRow,
		TableBodyCell
	} from 'flowbite-svelte';
	import Graph from '../graph.svelte';
	import { keepables, graphdata } from '../store';
	import EmptyVideo from '../EmptyVideo.svelte';

	let data = [
		['Battery:', '72%'],
		['Location:', '(31.9686, 99.9018)'],
		['Oxygen:', '96%'],
		['Heart Rate', '72bpm'],
		['CO2', '0.04%'],
		['Temperature', '20C']
	];

	const graphs = [
		{
			graphdata: [14.1, 13.9, 13.8, 13.7, 12.7, 12.3],
			lastDelta: '-12.3%',
			name: 'Battery',
			status: '12.3V'
		},
		{
			graphdata: [98.4, 97.3, 97, 96.5, 96, 96],
			lastDelta: '-1.7%',
			name: 'Oxygen Level',
			status: '96%'
		},
		{
			graphdata: [72, 72, 73, 74, 73, 72],
			lastDelta: '-1bpm',
			name: 'Heart Rate',
			status: '72bpm'
		},
		{
			graphdata: [0.04, 0.04, 0.04, 0.04, 0.04, 0.04],
			lastDelta: '0.01%',
			name: 'CO2 Level',
			status: '0.04%'
		}
	];
</script>

<div class="h-fit m-4">
	<br />
	<div class="border-b p-2 dark:border-gray-700">
		<Heading tag="h1">ASTRONAUT</Heading>
	</div>
	<!-- <Button color="alternative">Battery 1: &nbsp <Span highlight>72%</Span></Button>
    <Button color="alternative">Battery 2: &nbsp <Span highlight>76%</Span></Button>
    <Button color="alternative">Location: &nbsp (<Span highlight>31.9686</Span>,<Span highlight>99.9018</Span>)</Button> -->
	<div class="flex flex-row p-2 justify-start">
		<div class="flex flex-col min-w-0">
			<Heading tag="h3" class="pb-2">Stats</Heading>
			<div class="flex-row">
				{#each data as item}
					<Button
						color="alternative"
						on:click={() => keepables.addElement('ROVER', item)}
						class="m-1"
					>
						{item[0]} &nbsp
						<Span highlight>{item[1]}</Span>
					</Button>
				{/each}
			</div>
			<br />
			<Heading tag="h3" class="pb-2">Actions</Heading>
			<div class="flex-row">
				<Button class="m-0.5" color="dark">Collect Sample</Button>
				<Button class="m-0.5" color="dark">Drop Sample</Button>
				<Button class="m-0.5" color="dark">Take Picture</Button>
				<Button class="m-0.5" color="dark">Return</Button>
				<Button class="m-0.5">Self Destruct</Button>
			</div>
		</div>
		<div class="flex p-1 flex-grow ml-4 mr-16">
			<Card class="w-96">
				<Heading tag="h3" class="mb-5">Quick Access</Heading>
				<Table>
					<TableBody class="divide-y">
						<TableBodyRow>
							<TableBodyCell>Open Biometrics</TableBodyCell>
							<TableBodyCell>
								<a
									href="/tables"
									class="font-medium text-red-600 hover:underline dark:text-primary-500">Remove</a
								>
							</TableBodyCell>
						</TableBodyRow>
						<TableBodyRow>
							<TableBodyCell>Open Map</TableBodyCell>
							<TableBodyCell>
								<a
									href="/tables"
									class="font-medium text-red-600 hover:underline dark:text-primary-500">Remove</a
								>
							</TableBodyCell>
						</TableBodyRow>
						<TableBodyRow>
							<TableBodyCell>Ingress Procedure</TableBodyCell>
							<TableBodyCell>
								<a
									href="/tables"
									class="font-medium text-red-600 hover:underline dark:text-primary-500">Remove</a
								>
							</TableBodyCell>
						</TableBodyRow>
						<TableBodyRow>
							<TableBodyCell>Repair Procedure</TableBodyCell>
							<TableBodyCell>
								<a
									href="/tables"
									class="font-medium text-red-600 hover:underline dark:text-primary-500">Remove</a
								>
							</TableBodyCell>
						</TableBodyRow>
						<TableBodyRow>
							<TableBodyCell>Move Rover</TableBodyCell>
							<TableBodyCell>
								<a
									href="/tables"
									class="font-medium text-red-600 hover:underline dark:text-primary-500">Remove</a
								>
							</TableBodyCell>
						</TableBodyRow>
					</TableBody>
				</Table>
			</Card>
		</div>
	</div>
	<br />
	<Heading tag="h3">Cameras</Heading>
	<br />
	<div class="flex flex-row">
		<div class="p-2">
			<EmptyVideo name="Hololens" />
		</div>
	</div>
	<br />
	<br />
	<Heading tag="h3">Details</Heading>
	<br />
	<div class="flex flex-row flex-wrap">
		{#each graphs as graph}
			<button
				class="w-96 p-2"
				on:click={() => graphdata.addGraph('ASTRONAUT', graph.name, graph.graphdata)}
			>
				<Graph {...graph} />
			</button>
		{/each}
	</div>
</div>
