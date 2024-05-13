<script lang="ts">
	import { Timeline, TimelineItem, Button, Card, Heading, Input, Label } from 'flowbite-svelte';
	import { ArrowRightOutline, EditOutline, PlusSolid } from 'flowbite-svelte-icons';
	import ProcedureStep from './ProcedureStep.svelte';
	import { datastore } from '$lib/datastore';
	import { get, writable } from 'svelte/store';

	export let name: string;
	let editMode = false;
	let newname = name;
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
				name: newname,
				description: current.description,
				category: current.category,
				duration: current.duration,
				tasks: allSteps
			};
			const endpoint = 'http://localhost:8181/procedures/';
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
	{#if !editMode}
		<Button color="none" on:click={toggleEditMode}><EditOutline /></Button>
	{:else}
		<Button color="alternative" on:click={toggleEditMode}>Save</Button>
	{/if}
</div>
{#if editMode}
	<h1>Metadata</h1>
	<div class="ml-2 mb-4 flex flex-row">
		<Input bind:value={newname} />
		<Input bind:value={$datastore['procedure'][name]['category']} defaultClass="ml-2 mr-2" />
		<Input bind:value={$datastore['procedure'][name]['duration']} defaultClass="m-0" />
	</div>
{/if}
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
