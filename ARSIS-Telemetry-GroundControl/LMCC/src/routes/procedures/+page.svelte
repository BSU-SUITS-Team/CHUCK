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
		TableSearch
	} from 'flowbite-svelte';
	let searchTerm = '';
	$: procedureNames = Object.keys($datastore.procedure ?? {})	
	$: filteredItems = procedureNames.filter(
		(item) => item.toLowerCase().indexOf(searchTerm.toLowerCase()) !== -1
	);
</script>

<div class="w-auto h-fit m-4 mr-24">
	<Breadcrumb class="mb-4">
		<BreadcrumbItem href="/" home>Home</BreadcrumbItem>
		<BreadcrumbItem href="/procedures">Procedures</BreadcrumbItem>
	</Breadcrumb>
	<Heading tag="h2" class="mb-4">Procedures</Heading>
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
		</TableBody>
	</TableSearch>
</div>
