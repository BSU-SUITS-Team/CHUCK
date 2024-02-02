<script>
	import { Timeline, TimelineItem, Button, Card, Heading } from 'flowbite-svelte';
	import { ArrowRightOutline, PlusSolid } from 'flowbite-svelte-icons';
	import ProcedureStep from './ProcedureStep.svelte';

	export let name;

	let allSteps = [
		{
			title: 'Install Jetpack',
			description:
				'Lorem ipsum dolor sit amet, consectetur adipiscing elit, sed do eiusmod tempor incididunt ut labore et dolore magna aliqua. Ut enim ad minim veniam, quis nostrud exercitation ullamco laboris nisi ut aliquip ex ea commodo consequat. Duis aute irure dolor in reprehenderit in voluptate velit esse cillum dolore eu fugiat nulla pariatur. Excepteur sint occaecat cupidatat non proident, sunt in culpa qui officia deserunt mollit anim id est laborum',
			links: ['Battery < 20%', 'Oxygen > 50%'],
			problemLinks: ['Jetpack Recovery Procedure']
		},
		{
			title: 'Launch Missiles',
			description:
				'Lorem ipsum dolor sit amet, consectetur adipiscing elit, sed do eiusmod tempor incididunt ut labore et dolore magna aliqua. Ut enim ad minim veniam, quis nostrud exercitation ullamco laboris nisi ut aliquip ex ea commodo consequat. Duis aute irure dolor in reprehenderit in voluptate velit esse cillum dolore eu fugiat nulla pariatur. Excepteur sint occaecat cupidatat non proident, sunt in culpa qui officia deserunt mollit anim id est laborum'
		},
		{
			title: 'Propare Tofu',
			description:
				'Lorem ipsum dolor sit amet, consectetur adipiscing elit, sed do eiusmod tempor incididunt ut labore et dolore magna aliqua. Ut enim ad minim veniam, quis nostrud exercitation ullamco laboris nisi ut aliquip ex ea commodo consequat. Duis aute irure dolor in reprehenderit in voluptate velit esse cillum dolore eu fugiat nulla pariatur. Excepteur sint occaecat cupidatat non proident, sunt in culpa qui officia deserunt mollit anim id est laborum',
			substeps: [
				{
					title: 'Boil Water',
					date: 'Step 1',
					description:
						'Lorem ipsum dolor sit amet, consectetur adipiscing elit, sed do eiusmod tempor incididunt ut labore et dolore magna aliqua. Ut enim ad minim veniam, quis nostrud exercitation'
				},
				{
					title: '[REDACTED]',
					date: 'Step 1',
					description: '🔥'
				},
				{
					title: 'Serve',
					date: 'Step 1',
					description:
						'Lorem ipsum dolor sit amet, consectetur adipiscing elit, sed do eiusmod tempor incididunt ut labore et dolore magna aliqua. Ut enim ad minim veniam, quis nostrud exercitation'
				}
			]
		}
	];
</script>

<Heading tag="h2" class="mb-3">{name}</Heading>
<Timeline>
	{#each allSteps as step}
		<ProcedureStep {...step} date="Step {allSteps.indexOf(step) + 1}" />
	{/each}
</Timeline>
