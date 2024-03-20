<script lang="ts">
	import { Timeline, TimelineItem, Button, Card, Heading } from 'flowbite-svelte';
	import { ArrowRightOutline, EditOutline, PlusSolid } from 'flowbite-svelte-icons';
	import ProcedureStep from './ProcedureStep.svelte';
	import { datastore } from '$lib/datastore';
	import { writable } from 'svelte/store';

	export let name;
	let editMode = false;

	// let allSteps = [
	// 	{
	// 		title: 'Install Jetpack',
	// 		description:
	// 			'Lorem ipsum dolor sit amet, consectetur adipiscing elit, sed do eiusmod tempor incididunt ut labore et dolore magna aliqua. Ut enim ad minim veniam, quis nostrud exercitation ullamco laboris nisi ut aliquip ex ea commodo consequat. Duis aute irure dolor in reprehenderit in voluptate velit esse cillum dolore eu fugiat nulla pariatur. Excepteur sint occaecat cupidatat non proident, sunt in culpa qui officia deserunt mollit anim id est laborum',
	// 		links: ['Battery < 20%', 'Oxygen > 50%'],
	// 		problemLinks: ['Jetpack Recovery Procedure']
	// 	},
	// 	{
	// 		title: 'Launch Missiles',
	// 		description:
	// 			'Lorem ipsum dolor sit amet, consectetur adipiscing elit, sed do eiusmod tempor incididunt ut labore et dolore magna aliqua. Ut enim ad minim veniam, quis nostrud exercitation ullamco laboris nisi ut aliquip ex ea commodo consequat. Duis aute irure dolor in reprehenderit in voluptate velit esse cillum dolore eu fugiat nulla pariatur. Excepteur sint occaecat cupidatat non proident, sunt in culpa qui officia deserunt mollit anim id est laborum'
	// 	},
	// 	{
	// 		title: 'Propare Tofu',
	// 		description:
	// 			'Lorem ipsum dolor sit amet, consectetur adipiscing elit, sed do eiusmod tempor incididunt ut labore et dolore magna aliqua. Ut enim ad minim veniam, quis nostrud exercitation ullamco laboris nisi ut aliquip ex ea commodo consequat. Duis aute irure dolor in reprehenderit in voluptate velit esse cillum dolore eu fugiat nulla pariatur. Excepteur sint occaecat cupidatat non proident, sunt in culpa qui officia deserunt mollit anim id est laborum',
	// 		substeps: [
	// 			{
	// 				title: 'Boil Water',
	// 				date: 'Step 1',
	// 				description:
	// 					'Lorem ipsum dolor sit amet, consectetur adipiscing elit, sed do eiusmod tempor incididunt ut labore et dolore magna aliqua. Ut enim ad minim veniam, quis nostrud exercitation'
	// 			},
	// 			{
	// 				title: '[REDACTED]',
	// 				date: 'Step 1',
	// 				description: '🔥'
	// 			},
	// 			{
	// 				title: 'Serve',
	// 				date: 'Step 1',
	// 				description:
	// 					'Lorem ipsum dolor sit amet, consectetur adipiscing elit, sed do eiusmod tempor incididunt ut labore et dolore magna aliqua. Ut enim ad minim veniam, quis nostrud exercitation'
	// 			}
	// 		]
	// 	}
	// ];
	let allSteps = [];
	function setSteps() {
		if (editMode) { return }
		allSteps = $datastore['procedure']
			? $datastore['procedure'][name].tasks
			: [{ title: 'Nothing found', description: null }];
	}
	$: currentSteps = writable(allSteps, () => {
		return datastore.subscribe(setSteps);
	});
	function toggleEditMode() {
		console.log(currentSteps);
		editMode = !editMode;
	}
</script>

<div class="flex justify-between">
	<Heading tag="h2" class="mb-3">{name}</Heading>
	<Button color="none" on:click={toggleEditMode}><EditOutline /></Button>
</div>
<Timeline>
	{#each $currentSteps as step}
		<ProcedureStep {...step} {editMode} date="Step {allSteps.indexOf(step) + 1}" />
	{/each}
</Timeline>
