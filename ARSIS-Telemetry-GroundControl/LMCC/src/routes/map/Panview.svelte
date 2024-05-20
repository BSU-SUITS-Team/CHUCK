<script lang="ts">
	import { onMount } from 'svelte';
	import PinButton from './PinButton.svelte';
	import BoxButton from './BoxButton.svelte';
	import PathButton from './PathButton.svelte';
	import { datastore } from '$lib/datastore';
	import { Button, Input } from 'flowbite-svelte';
	import { XCircleOutline } from 'flowbite-svelte-icons';

	let img;
	let naturalHeight;
	let naturalWidth;
	let scale = 1;
	let offsetX = 0;
	let pinProximity = 15;
	let offsetY = 0;
	let isPanning = false;
	let startX, startY;
	// ex. [{type: 'red', x: 100, y: 100}, {type: 'blue', x: 200, y: 200}]
	let pins = [];
	let isPlacingPin = false;
	let buttons = [true, true, true, true];
	let newname = '';

	export let image;
	export let initalSize = 1;
	export let minScale = 0.1;
	export let maxScale = 10;
	export let xRange = [0, 9999999];
	export let yRange = [0, 9999999];
	export let initalPosition = [0, 0];
	let editingPin = null;

	datastore.subscribe(loadPins);

	function loadPins(fromData) {
		if (!fromData['pins']) {
			return;
		}
		let newPins = [];
		let pinList = Object.keys(fromData['pins']);
		for (let i = 0; i < pinList.length; i++) {
			console.log(pinList[i]);
			const { x, y, id, name = '' } = fromData['pins'][pinList[i]]['properties'];
			newPins.push({ type: 'red', x, y, name, id });
		}
		pins = newPins;
	}

	function distanceBetween(x1, y1, x2, y2) {
		return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5;
	}

	function handleWheel(event) {
		const delta = event.deltaY < 0 ? 1.1 : 0.9;
		const rect = event.currentTarget.getBoundingClientRect();
		const x = event.clientX - rect.left;
		const y = event.clientY - rect.top;

		const newScale = Math.max(minScale, Math.min(maxScale, scale * delta));

		const newOffsetX = ((offsetX - x) * newScale) / scale + x;
		const newOffsetY = ((offsetY - y) * newScale) / scale + y;

		xRange = [xRange[0] * newScale, xRange[1] * newScale];
		yRange = [yRange[0] * newScale, yRange[1] * newScale];

		offsetX = Math.min(xRange[1], Math.max(xRange[0], newOffsetX));
		offsetY = Math.min(yRange[1], Math.max(yRange[0], newOffsetY));

		scale = newScale;
	}

	function updatePinName() {
		let { x, y, id } = pins[editingPin];
		addPin(x, y, id, newname);
		newname = '';
		editingPin = null;
	}

	async function addPin(x, y, id = null, name = '') {
		const url = 'http://localhost:8181/navigation/pins';
		const data = {
			x,
			y,
			lat: 0,
			lon: 0,
			properties: id == null ? { name } : { id: id, name }
		};

		try {
			const response = await fetch(url, {
				method: 'POST',
				headers: {
					accept: 'application/json',
					'Content-Type': 'application/json'
				},
				body: JSON.stringify(data)
			});

			if (response.ok) {
				const result = await response.json();
				console.log('Pin added successfully:', result);
				return result;
			} else {
				console.error('Error adding pin:', response.status);
				throw new Error('Failed to add pin');
			}
		} catch (error) {
			console.error('Error adding pin:', error);
			throw error;
		}
	}

	function handleMouseDown(event) {
		const x = event.clientX - event.currentTarget.offsetLeft;
		const y = event.clientY - event.currentTarget.offsetTop;
		const correctedX = ((x - offsetX) / scale / img.clientWidth) * naturalWidth;
		const correctedY = ((y - offsetY) / scale / img.clientHeight) * naturalHeight;

		if (isPlacingPin) {
			pins = [...pins, { type: isPlacingPin, x: correctedX, y: correctedY }];
			addPin(correctedX, correctedY);
			isPlacingPin = false;
			buttons = buttons.map(() => true);
			return;
		}

		//collison detection
		for (let i = 0; i < pins.length; i++) {
			let pin = pins[i];
			if (
				distanceBetween(
					x,
					y,
					(pin.x / naturalWidth) * (img.clientWidth * scale) + offsetX,
					(pin.y / naturalHeight) * (img.clientHeight * scale) + offsetY
				) < pinProximity
			) {
				pinClicked(i);
				return;
			}
		}

		isPanning = true;
		startX = event.clientX - offsetX;
		startY = event.clientY - offsetY;
	}

	function startPlacingPin(type, index) {
		isPlacingPin = type;
		buttons = buttons.map((_, i) => i == index);
	}

	function handleMouseMove(event) {
		if (isPanning) {
			const newOffsetX = event.clientX - startX;
			const newOffsetY = event.clientY - startY;

			offsetX = Math.min(xRange[1], Math.max(xRange[0], newOffsetX));
			offsetY = Math.min(yRange[1], Math.max(yRange[0], newOffsetY));
		}
	}

	function handleMouseUp() {
		isPanning = false;
	}

	function handleDragStart(event) {
		event.preventDefault();
	}

	function pinClicked(pinindex) {
		console.log(`PIN CLICKED at ${pinindex}`);
		editingPin = pinindex;
	}

	onMount(() => {
		scale = initalSize;
		offsetX = initalPosition[0];
		offsetY = initalPosition[1];
	});
</script>

<!-- svelte-ignore a11y-no-static-element-interactions -->
<div
	on:wheel={handleWheel}
	on:mousedown={handleMouseDown}
	on:mousemove={handleMouseMove}
	on:mouseup={handleMouseUp}
	on:mouseleave={handleMouseUp}
	on:dragstart={handleDragStart}
	style="
      overflow: hidden;
      width: 100%;
      height: 100%;
      position: relative;
    "
>
	{#each pins as pin}
		<div
			class="absolute z-10"
			style="
	transform: translate3d({(pin.x / naturalWidth) * (img.clientWidth * scale) + offsetX}px, {(pin.y /
				naturalHeight) *
				(img.clientHeight * scale) +
				offsetY}px, 0);
	transform-origin: 0 0;
  "
		>
			<PinButton color={pin.type} name={pin.name} move />
		</div>
	{/each}
	<div class="absolute z-10 top-5 w-full h-full select-none">
		<div class="flex justify-center">
			<div class="mb-4 p-2 shadow-xl rounded-lg flex flex-row bg-white w-fit dark:bg-gray-800">
				<BoxButton />
				<PathButton />
			</div>
			<div
				class="mb-4 p-2 shadow-xl rounded-lg flex flex-row bg-white w-fit dark:bg-gray-800 ml-1 mr-1"
			>
				<PinButton
					color="black"
					onclick={() => startPlacingPin('black', 0)}
					bind:bright={buttons[0]}
				/>
				<PinButton color="red" onclick={() => startPlacingPin('red', 1)} bind:bright={buttons[1]} />
				<PinButton
					color="blue"
					onclick={() => startPlacingPin('blue', 2)}
					bind:bright={buttons[2]}
				/>
				<PinButton
					color="green"
					onclick={() => startPlacingPin('green', 3)}
					bind:bright={buttons[3]}
				/>
			</div>
		</div>
	</div>
	<img
		src={image}
		bind:naturalHeight
		bind:naturalWidth
		bind:this={img}
		alt="a giant spaceship"
		style="
		transform: translate3d({offsetX}px, {offsetY}px, 0) scale({scale});
		transform-origin: 0 0;
		position: relative;
		max-width: 100%;
		max-height: 100%;
		"
		class="top-0 left-0"
	/>
</div>

{#if editingPin != null}
	<div
		class="w-96 bg-white absolute top-1/2 left-1/2 -translate-x-1/2 -translate-y-1/2 rounded-lg shadow-xl p-6 z-50"
	>
		Pin Label
		<Input bind:value={newname} />
		<br />
		<div class="flex justify-between">
			<!-- 		<Button>Delete Pin</Button> It is currently not possible to remove anything though events -->
			<Button color="alternative" on:click={updatePinName}>Confirm</Button>
		</div>
	</div>
{/if}
