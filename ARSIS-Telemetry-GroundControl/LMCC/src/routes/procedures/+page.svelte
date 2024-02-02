<script>
	import {
		Breadcrumb,
		BreadcrumbItem,
		Heading,
		Span,
		Table,
		TableHead,
		TableHeadCell,
		TableBody,
		TableBodyRow,
		TableBodyCell,
		TableSearch
	} from 'flowbite-svelte';
	import { Timeline, TimelineItem, Button, Card } from 'flowbite-svelte';
	import { ArrowRightOutline, PlusSolid } from 'flowbite-svelte-icons';
	let searchTerm = '';
	let prodecureNames = [
		'Test Procedure 1',
		'How to make a sandwich',
		'Emergency Snack Procedure',
		'Deploy Parachute',
		'Do EVA',
		'Test Procedure 5'
	];
	$: filteredItems = prodecureNames.filter(
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
		<TableBody class="divide-y">
			{#each filteredItems as prcedure}
				<TableBodyRow>
					<TableBodyCell>
						<a href="/procedures/{prcedure}" class="flex">
							{prcedure}
						</a></TableBodyCell
					>
					<TableBodyCell>Test Procedures</TableBodyCell>
					<TableBodyCell>-</TableBodyCell>
				</TableBodyRow>
			{/each}
		</TableBody>
	</TableSearch>
</div>
