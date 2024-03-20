<script lang="ts">
	import { Timeline, TimelineItem, Button, Card, Heading } from 'flowbite-svelte';
	import { ArrowRightOutline, EditOutline, PlusSolid } from 'flowbite-svelte-icons';
	import ProcedureStep from './ProcedureStep.svelte';
	import { datastore } from '$lib/datastore';
	import { get, writable } from 'svelte/store';

	export let name: string;
	let editMode = false;

	let allSteps = [];
	function setSteps() {
		if (editMode) {
			return;
		}
		allSteps = $datastore['procedure']
			? $datastore['procedure'][name].tasks
			: [{ title: 'Nothing found', description: null }];
	}
	
	datastore.subscribe(setSteps);
	$: _ = console.log(allSteps)

	function toggleEditMode() {
		if (editMode == true) {
			//send the post requests to update the procedures
			const data = { name: name, summary: '', tasks: allSteps };
			const endpoint = 'http://localhost:8181/procedures/';
			console.log(data);
			fetch(endpoint, {
				method: 'POST',
				headers: {
					'Content-Type': 'application/json'
				},
				body: JSON.stringify(data)
			}).catch((error) => console.log(error));
		}
		editMode = !editMode;
	}
</script>

<div class="flex justify-between">
	<Heading tag="h2" class="mb-3">{name}</Heading>
	<Button color="none" on:click={toggleEditMode}><EditOutline /></Button>
</div>
<Timeline>
	{#if allSteps}
		{#each allSteps as step, i}
			<ProcedureStep bind:title={step.name} bind:description={step.description} {editMode} date="Step {i + 1}" />
		{/each}
	{/if}
</Timeline>
