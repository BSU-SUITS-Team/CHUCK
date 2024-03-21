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

	function toggleEditMode() {
		if (editMode == true) {
			//send the post requests to update the procedures
			let current = $datastore['procedure'][name];
			const data = {
				name: name,
				description: current.dessciption,
				category: current.category,
				duration: current.duration,
				tasks: allSteps
			};
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

	function addNewStepAfter(position: number) {
		allSteps.splice(position + 1, 0, { name: 'New Step', description: '', steps: [] });
		allSteps = allSteps;
	}
	function removeStepAt(position: number) {
		allSteps.splice(position, 1);
		allSteps = allSteps;
	}
</script>

<div class="flex justify-between">
	<Heading tag="h2" class="mb-3">{name}</Heading>
	<Button color="none" on:click={toggleEditMode}><EditOutline /></Button>
</div>
<Timeline>
	{#if allSteps}
		{#each allSteps as step, i}
			<ProcedureStep
				bind:title={step.name}
				bind:description={step.description}
				bind:steps={step.steps}
				createNewStep={() => {
					addNewStepAfter(i);
				}}
				removeThisStep={() => {
					removeStepAt(i);
				}}
				{editMode}
				date="Step {i + 1}"
			/>
		{/each}
	{/if}
</Timeline>
