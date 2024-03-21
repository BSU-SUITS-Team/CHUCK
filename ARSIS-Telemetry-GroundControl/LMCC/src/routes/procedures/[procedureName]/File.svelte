<script>
	let fileInput;
	export let encodedString = '';

	async function handleFileSelect() {
		const file = fileInput.files[0];
		if (file) {
			const reader = new FileReader();
			reader.onload = () => {
				const base64String = btoa(reader.result);
				encodedString = base64String;
			};
			reader.readAsBinaryString(file);
		}
	}
</script>

<div
	class="w-full rounded-lg bg-gray-200 h-10 border border-gray-300 p-3 text-sm mt-4 overflow-clip"
>
	<input
		type="file"
		bind:this={fileInput}
		on:change={handleFileSelect}
		class="-translate-y-3 -translate-x-2"
	/>
</div>
