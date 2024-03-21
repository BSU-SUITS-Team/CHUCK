<script>
	import { goto } from '$app/navigation';
	import { Input, Label, Button } from 'flowbite-svelte';

	let name = '';
	let description = '';

	function createNewProcedure() {
		const data = {
			name: name,
			description: description,
			category: 'New',
			duration: 'Empty Procedure',
			tasks: [{ name: '', description: '', steps: [] }]
		};
		const endpoint = 'http://localhost:8181/procedures/';
		fetch(endpoint, {
			method: 'POST',
			headers: {
				'Content-Type': 'application/json'
			},
			body: JSON.stringify(data)
		})
			.then(goto('/procedures'))
			.catch((error) => console.log(error));
	}
</script>

<div class="w-full h-full flex items-center justify-center">
	<div class="w-96 border border-gray-300 shadow-lg rounded-lg p-4 bg-gray-50">
		<h1>Create New Procedure</h1>
		<br />
		<Label>Procedure Name</Label>
		<Input bind:value={name} />
		<Label>Description</Label>
		<Input bind:value={description} />
		<Button class="mt-2 w-full" on:click={createNewProcedure}>Create</Button>
	</div>
</div>
