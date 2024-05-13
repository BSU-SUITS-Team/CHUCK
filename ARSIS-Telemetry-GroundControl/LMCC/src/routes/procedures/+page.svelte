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

	let testprocedure = {
		name: 'This is a test procedure',
		description: 'Somthing',
		category: 'Test',
		duration: '10 mins',
		tasks: [{ name: 'This is a test', description: null, steps: [] }]
	};

	let stagedProcedures = [testprocedure];
	$: filteredNewProcedures = stagedProcedures.filter((element) => !procedureNames.includes(element.name));

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
