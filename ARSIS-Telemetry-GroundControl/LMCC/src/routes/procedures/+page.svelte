<script lang="ts">
	import { datastore } from '$lib/datastore';
	import {
		Breadcrumb,
		BreadcrumbItem,
		Heading,
		TableHead,
		TableHeadCell,
		TableBody,
		TableBodyRow,
		TableBodyCell,
		TableSearch,
		Button
	} from 'flowbite-svelte';
	let searchTerm = '';
	$: procedureNames = Object.keys($datastore.procedure ?? {});
	$: filteredItems = procedureNames.filter(
		(item) => item.toLowerCase().indexOf(searchTerm.toLowerCase()) !== -1
	);

	let stagedProcedures = [
		{
			name: 'Heart Rate High',
			description: '',
			category: 'Emergency',
			duration: '2 mins',
			tasks: [
				{ name: 'Verify heart rate on Biometrics display.', description: null, steps: [] },
				{
					name: 'Does crewmember feel their heart rate to be high? (Yes/No)',
					description: null,
					steps: [
						{
							body: 'YES: Slow down movements movements to get heart rate below 120.',
							nextTask: [],
							type: 'text'
						},
						{
							body: 'NO: Sensor failure - continue as desired. (Exit procedure)',
							nextTask: [],
							type: 'text'
						}
					]
				}
			]
		},
		{
			name: 'Suit Pressure O2',
			description: '',
			category: 'Emergency',
			duration: '2 mins',
			tasks: [
				{
					name: 'Is Suit pressure > 4.1 or Suit Pressure < 3.5 (Yes/No)',
					description: null,
					steps: []
				},
				{
					name: 'Yes',
					description: null,
					steps: [
						{
							body: 'DCM OXY -> SEC',
							nextTask: [],
							type: 'text'
						},
						{
							body: 'Verify suit pressure back in range (4.1 > Pressure > 3.5)',
							nextTask: [],
							type: 'text'
						},
						{
							body: 'If it is: (Exit procedure)',
							nextTask: [],
							type: 'text'
						},
						{
							body: 'If it is not: Go to procedure Abort EVA',
							nextTask: [],
							type: 'text'
						}
					]
				},
				{
					name: 'No',
					description: 'Faulty sensor. (Exit Procedure)',
					steps: []
				}
			]
		},
		{
			name: 'Suit Pressure CO2',
			description: '',
			category: 'Emergency',
			duration: '2 mins',
			tasks: [
				{ name: 'Is Suit CO2 pressure > 0.1 (Yes/No)', description: null, steps: [] },
				{
					name: 'Yes',
					description: null,
					steps: [
						{
							body: 'DCM CO2 -> Alternate system (A or B)',
							nextTask: [],
							type: 'text'
						},
						{
							body: 'Verify CO2 decreasing (Yes/No).',
							nextTask: [],
							type: 'text'
						},
						{
							body: 'If it is: Continue EVA, monitor CO2 (Exit procedure)',
							nextTask: [],
							type: 'text'
						},
						{
							body: 'If it is not: Go to procedure Term EVA',
							nextTask: [],
							type: 'text'
						}
					]
				},
				{
					name: 'No',
					description: 'Faulty sensor. (Exit Procedure)',
					steps: []
				}
			]
		},
		{
			name: 'Helmet Pressure CO2',
			description: '',
			category: 'Emergency',
			duration: '2 mins',
			tasks: [
				{ name: 'Verify Fan RPM > 20,000? (Yes/No)', description: null, steps: [] },
				{
					name: 'Yes',
					description: null,
					steps: [
						{
							body: 'DCU Fan -> Alternate option (Exit procedure)',
							nextTask: [],
							type: 'text'
						}
					]
				},
				{
					name: 'No',
					description: 'Go to procedure SUIT PRESSURE CO2',
					steps: []
				}
			]
		},
		{
			name: 'Fan X RPM Low (<20,000)',
			description: '',
			category: 'Emergency',
			duration: '2 mins',
			tasks: [
				{ name: 'Verify Fan RPM > 20,000? (Yes/No)', description: null, steps: [] },
				{
					name: 'Yes',
					description: 'Sensor failure (Exit procedure)',
					steps: []
				},
				{
					name: 'No',
					description: 'DCU Fan -> Alternate option',
					steps: []
				}
			]
		},
		{
			name: 'Scrubber X CO2 Storage (>60)',
			description: '',
			category: 'Emergency',
			duration: '2 mins',
			tasks: [{ name: 'DCU CO2 -> Alternate option', description: null, steps: [] }]
		},
		{
			name: 'Temperature (>90)',
			description: '',
			category: 'Emergency',
			duration: '2 mins',
			tasks: [
				{ name: 'Are you hot? (Yes/No)', description: null, steps: [] },
				{
					name: 'Yes',
					description: null,
					steps: [
						{
							body: 'Slow downn and rest to let temperature drow',
							nextTask: [],
							type: 'text'
						},
						{
							body: 'Monitor temp',
							nextTask: [],
							type: 'text'
						},
						{
							body: 'Continue at slower pace once temp is below 90',
							nextTask: [],
							type: 'text'
						}
					]
				},
				{
					name: 'No',
					description: 'Faulty sensor. (Exit Procedure)',
					steps: []
				}
			]
		},
		{
			name: 'Term EVA',
			description: '',
			category: 'Emergency',
			duration: '2 mins',
			tasks: [
				{ name: 'Clean up worksite', description: null, steps: [] },
				{
					name: 'Go to procedure ABORT EVA',
					description: null,
					steps: []
				}
			]
		},
		{
			name: 'Abort EVA',
			description: '',
			category: 'Emergency',
			duration: '2 mins',
			tasks: [
				{ name: 'Translate to Airlock', description: null, steps: [] },
				{
					name: 'Go to Procedure AIRLOCK INGRESS',
					description: null,
					steps: []
				}
			]
		}
	];
	$: filteredNewProcedures = stagedProcedures.filter(
		(element) => !procedureNames.includes(element.name)
	);

	function createNewProcedure(procedure: Object) {
		const endpoint = 'http://localhost:8181/procedures/';
		fetch(endpoint, {
			method: 'POST',
			headers: {
				'Content-Type': 'application/json'
			},
			body: JSON.stringify(procedure)
		}).catch((error) => console.log(error));
	}
</script>

<div class="w-auto h-fit m-4 mr-24">
	<Breadcrumb class="mb-4">
		<BreadcrumbItem href="/" home>Home</BreadcrumbItem>
		<BreadcrumbItem href="/procedures">Procedures</BreadcrumbItem>
	</Breadcrumb>
	<div class="flex justify-between mb-4">
		<Heading tag="h2">Procedures</Heading>
		<Button color="alternative" href="/new/procedure">New</Button>
	</div>
	<TableSearch hoverable bind:inputValue={searchTerm}>
		<TableHead>
			<TableHeadCell>Procedure Name</TableHeadCell>
			<TableHeadCell>Category</TableHeadCell>
			<TableHeadCell>Duration</TableHeadCell>
		</TableHead>
		<TableBody>
			{#each filteredItems as prcedure}
				<TableBodyRow>
					<TableBodyCell>
						<a href="/procedures/{prcedure}" class="flex">
							{prcedure}
						</a></TableBodyCell
					>
					<TableBodyCell>{$datastore['procedure'][prcedure]['category']}</TableBodyCell>
					<TableBodyCell>{$datastore['procedure'][prcedure]['duration']}</TableBodyCell>
				</TableBodyRow>
			{/each}
			{#each filteredNewProcedures as proc}
				<TableBodyRow color="custom" style="background-color: rgb(235, 235, 235);">
					<TableBodyCell>{proc.name}</TableBodyCell>
					<TableBodyCell>Staged</TableBodyCell>
					<TableBodyCell>
						<div class="flex justify-between w-full items-center m-0">
							<p>{proc.duration}</p>
							<Button
								color="dark"
								on:click={() => {
									createNewProcedure(proc);
								}}
							>
								Send Procedure
							</Button>
						</div>
					</TableBodyCell>
				</TableBodyRow>
			{/each}
		</TableBody>
	</TableSearch>
</div>
