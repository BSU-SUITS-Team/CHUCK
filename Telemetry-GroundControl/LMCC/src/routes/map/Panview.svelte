<script lang="ts">
	import { onDestroy, onMount } from 'svelte';
	import {
		HomeOutline,
		ZoomInOutline,
		ZoomOutOutline
	} from 'flowbite-svelte-icons';
	import PinButton from './PinButton.svelte';
	import BoxButton from './BoxButton.svelte';
	import PathButton from './PathButton.svelte';
	import { datastore } from '$lib/datastore';
	import { Button, Input } from 'flowbite-svelte';

	type Pin = {
		type: string;
		x: number;
		y: number;
		id?: string | number;
		name?: string;
	};

	let viewport: HTMLDivElement;
	let img: HTMLImageElement;
	let naturalHeight = 0;
	let naturalWidth = 0;
	let scale = 1;
	let offsetX = 0;
	let offsetY = 0;
	let pinProximity = 18;
	let isPanning = false;
	let startX = 0;
	let startY = 0;
	let startOffsetX = 0;
	let startOffsetY = 0;
	let pins: Pin[] = [];
	let isPlacingPin: string | false = false;
	let buttons = [true, true, true, true];
	let newname = '';
	let editingPin: number | null = null;
	let hasInitialized = false;
	let resizeObserver: ResizeObserver | undefined;

	export let image: string;
	export let initalSize = 1.05;
	export let minScale = 0.12;
	export let maxScale = 8;
	export let fitPadding = 36;
	export let panMargin = 64;

	const unsubscribe = datastore.subscribe(loadPins);
	onDestroy(unsubscribe);

	function loadPins(fromData: Record<string, any>) {
		if (!fromData['pins']) return;

		let newPins: Pin[] = [];
		let pinList = Object.keys(fromData['pins']);
		for (let i = 0; i < pinList.length; i++) {
			const { x, y, id, name = '' } = fromData['pins'][pinList[i]]['properties'];
			newPins.push({ type: 'red', x, y, name, id });
		}
		pins = newPins;
	}

	function distanceBetween(x1: number, y1: number, x2: number, y2: number) {
		return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5;
	}

	function viewportRect() {
		return viewport.getBoundingClientRect();
	}

	function fitScale() {
		if (!viewport || !naturalWidth || !naturalHeight) return 1;

		const widthScale = (viewport.clientWidth - fitPadding * 2) / naturalWidth;
		const heightScale = (viewport.clientHeight - fitPadding * 2) / naturalHeight;
		return Math.max(0.01, Math.min(widthScale, heightScale));
	}

	function clampScale(nextScale: number) {
		const minFitScale = fitScale() * 0.8;
		return Math.max(Math.min(minScale, minFitScale), Math.min(maxScale, nextScale));
	}

	function clampOffset(x: number, y: number, nextScale = scale) {
		if (!viewport || !naturalWidth || !naturalHeight) return { x, y };

		const mapWidth = naturalWidth * nextScale;
		const mapHeight = naturalHeight * nextScale;
		const viewWidth = viewport.clientWidth;
		const viewHeight = viewport.clientHeight;

		const horizontal =
			mapWidth <= viewWidth - panMargin * 2
				? { min: (viewWidth - mapWidth) / 2, max: (viewWidth - mapWidth) / 2 }
				: { min: Math.min(panMargin, viewWidth - mapWidth - panMargin), max: panMargin };
		const vertical =
			mapHeight <= viewHeight - panMargin * 2
				? { min: (viewHeight - mapHeight) / 2, max: (viewHeight - mapHeight) / 2 }
				: { min: Math.min(panMargin, viewHeight - mapHeight - panMargin), max: panMargin };

		return {
			x: Math.min(horizontal.max, Math.max(horizontal.min, x)),
			y: Math.min(vertical.max, Math.max(vertical.min, y))
		};
	}

	function centerMap(nextScale = scale) {
		const centeredX = (viewport.clientWidth - naturalWidth * nextScale) / 2;
		const centeredY = (viewport.clientHeight - naturalHeight * nextScale) / 2;
		return clampOffset(centeredX, centeredY, nextScale);
	}

	function resetView() {
		if (!viewport || !naturalWidth || !naturalHeight) return;

		const nextScale = clampScale(fitScale() * initalSize);
		const centered = centerMap(nextScale);
		scale = nextScale;
		offsetX = centered.x;
		offsetY = centered.y;
		hasInitialized = true;
	}

	function handleImageLoad() {
		naturalWidth = img.naturalWidth;
		naturalHeight = img.naturalHeight;
		resetView();
	}

	function zoomAt(screenX: number, screenY: number, zoomFactor: number) {
		if (!naturalWidth || !naturalHeight) return;

		const nextScale = clampScale(scale * zoomFactor);
		const mapX = (screenX - offsetX) / scale;
		const mapY = (screenY - offsetY) / scale;
		const nextOffset = clampOffset(screenX - mapX * nextScale, screenY - mapY * nextScale, nextScale);

		scale = nextScale;
		offsetX = nextOffset.x;
		offsetY = nextOffset.y;
	}

	function zoomAtCenter(zoomFactor: number) {
		zoomAt(viewport.clientWidth / 2, viewport.clientHeight / 2, zoomFactor);
	}

	function handleWheel(event: WheelEvent) {
		const rect = viewportRect();
		const x = event.clientX - rect.left;
		const y = event.clientY - rect.top;
		const zoomFactor = Math.exp(-event.deltaY * 0.0008);

		zoomAt(x, y, zoomFactor);
	}

	function screenToMap(clientX: number, clientY: number) {
		const rect = viewportRect();
		return {
			screenX: clientX - rect.left,
			screenY: clientY - rect.top,
			mapX: (clientX - rect.left - offsetX) / scale,
			mapY: (clientY - rect.top - offsetY) / scale
		};
	}

	function updatePinName() {
		if (editingPin == null) return;

		let { x, y, id } = pins[editingPin];
		addPin(x, y, id, newname);
		newname = '';
		editingPin = null;
	}

	async function addPin(x: number, y: number, id: string | number | null = null, name = '') {
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
				return result;
			} else {
				throw new Error('Failed to add pin');
			}
		} catch (error) {
			console.error('Error adding pin:', error);
			throw error;
		}
	}

	function handlePointerDown(event: PointerEvent) {
		if ((event.target as HTMLElement).closest('[data-map-control]')) return;

		const point = screenToMap(event.clientX, event.clientY);

		if (isPlacingPin) {
			pins = [...pins, { type: isPlacingPin, x: point.mapX, y: point.mapY }];
			addPin(point.mapX, point.mapY);
			isPlacingPin = false;
			buttons = buttons.map(() => true);
			return;
		}

		for (let i = 0; i < pins.length; i++) {
			let pin = pins[i];
			if (distanceBetween(point.screenX, point.screenY, pin.x * scale + offsetX, pin.y * scale + offsetY) < pinProximity) {
				pinClicked(i);
				return;
			}
		}

		isPanning = true;
		startX = event.clientX;
		startY = event.clientY;
		startOffsetX = offsetX;
		startOffsetY = offsetY;
		viewport.setPointerCapture(event.pointerId);
	}

	function startPlacingPin(type: string, index: number) {
		isPlacingPin = type;
		buttons = buttons.map((_, i) => i == index);
	}

	function handlePointerMove(event: PointerEvent) {
		if (!isPanning) return;

		const nextOffset = clampOffset(
			startOffsetX + event.clientX - startX,
			startOffsetY + event.clientY - startY
		);
		offsetX = nextOffset.x;
		offsetY = nextOffset.y;
	}

	function handlePointerUp(event: PointerEvent) {
		isPanning = false;
		if (viewport?.hasPointerCapture(event.pointerId)) {
			viewport.releasePointerCapture(event.pointerId);
		}
	}

	function handleResize() {
		if (!viewport || !naturalWidth || !naturalHeight) return;

		if (!hasInitialized) {
			resetView();
			return;
		}

		const centerX = viewport.clientWidth / 2;
		const centerY = viewport.clientHeight / 2;
		const mapX = (centerX - offsetX) / scale;
		const mapY = (centerY - offsetY) / scale;
		const nextOffset = clampOffset(centerX - mapX * scale, centerY - mapY * scale);
		offsetX = nextOffset.x;
		offsetY = nextOffset.y;
	}

	function handleDragStart(event: DragEvent) {
		event.preventDefault();
	}

	function pinClicked(pinindex: number) {
		editingPin = pinindex;
		newname = pins[pinindex].name ?? '';
	}

	onMount(() => {
		resizeObserver = new ResizeObserver(handleResize);
		resizeObserver.observe(viewport);
	});
</script>

<!-- svelte-ignore a11y-no-static-element-interactions -->
<div
	bind:this={viewport}
	on:wheel|preventDefault={handleWheel}
	on:pointerdown={handlePointerDown}
	on:pointermove={handlePointerMove}
	on:pointerup={handlePointerUp}
	on:pointercancel={handlePointerUp}
	on:dragstart={handleDragStart}
	class:cursor-grabbing={isPanning}
	class:cursor-crosshair={isPlacingPin}
	class="relative h-full w-full cursor-grab overflow-hidden bg-slate-100"
	style="touch-action: none;"
>
	<img
		src={image}
		bind:this={img}
		on:load={handleImageLoad}
		alt="Rock yard map"
		draggable="false"
		style="
			transform: translate3d({offsetX}px, {offsetY}px, 0) scale({scale});
			transform-origin: 0 0;
			width: {naturalWidth ? `${naturalWidth}px` : 'auto'};
			height: {naturalHeight ? `${naturalHeight}px` : 'auto'};
		"
		class="absolute left-0 top-0 max-h-none max-w-none select-none rounded-sm shadow-sm"
	/>

	{#each pins as pin}
		<div
			class="pointer-events-none absolute z-10"
			style="
				transform: translate3d({pin.x * scale + offsetX}px, {pin.y * scale + offsetY}px, 0);
				transform-origin: 0 0;
			"
		>
			<PinButton color={pin.type} name={pin.name} move />
		</div>
	{/each}

	<div class="pointer-events-none absolute inset-x-0 top-4 z-20 flex justify-center px-4">
		<div
			data-map-control
			class="pointer-events-auto flex max-w-full flex-wrap items-center justify-center gap-2 rounded-lg border border-slate-200 bg-white/95 p-2 shadow-lg"
		>
			<div class="flex items-center rounded-md border border-slate-200 bg-slate-50 p-1">
				<button
					type="button"
					class="rounded p-2 text-slate-600 transition hover:bg-white hover:text-slate-950"
					aria-label="Zoom out"
					title="Zoom out"
					on:click={() => zoomAtCenter(1 / 1.2)}
				>
					<ZoomOutOutline class="h-5 w-5" />
				</button>
				<button
					type="button"
					class="rounded p-2 text-slate-600 transition hover:bg-white hover:text-slate-950"
					aria-label="Reset map"
					title="Reset map"
					on:click={resetView}
				>
					<HomeOutline class="h-5 w-5" />
				</button>
				<button
					type="button"
					class="rounded p-2 text-slate-600 transition hover:bg-white hover:text-slate-950"
					aria-label="Zoom in"
					title="Zoom in"
					on:click={() => zoomAtCenter(1.2)}
				>
					<ZoomInOutline class="h-5 w-5" />
				</button>
			</div>

			<div class="flex items-center rounded-md border border-slate-200 bg-slate-50 p-1">
				<BoxButton />
				<PathButton />
			</div>

			<div class="flex items-center rounded-md border border-slate-200 bg-slate-50 p-1">
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
</div>

{#if editingPin != null}
	<div
		class="absolute left-1/2 top-1/2 z-50 w-96 -translate-x-1/2 -translate-y-1/2 rounded-lg bg-white p-6 shadow-xl"
	>
		<label class="text-sm font-semibold text-slate-800" for="pin-label">Pin Label</label>
		<Input id="pin-label" bind:value={newname} class="mt-2" />
		<div class="mt-4 flex justify-end">
			<Button color="alternative" on:click={updatePinName}>Confirm</Button>
		</div>
	</div>
{/if}
