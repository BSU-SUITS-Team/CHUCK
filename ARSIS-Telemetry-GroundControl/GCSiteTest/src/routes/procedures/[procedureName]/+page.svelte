<script>
	import { Breadcrumb, BreadcrumbItem, Heading, Span } from 'flowbite-svelte';
	import Procedure from './Procedure.svelte';

	/** @type {import('./$types').PageData} */
	export let data;
</script>

<div class="w-auto h-fit m-4 mr-24">
	<Breadcrumb class="mb-4">
		<BreadcrumbItem href="/" home>Home</BreadcrumbItem>
		<BreadcrumbItem href="/procedures">Procedures</BreadcrumbItem>
		<BreadcrumbItem href="/procedures/{data.name}">{data.name}</BreadcrumbItem>
	</Breadcrumb>
	<Procedure name={data.name} />
</div>
