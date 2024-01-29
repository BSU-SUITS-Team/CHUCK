<script>
	export let fullRange = [0, 10]; // example of a range instead of a singular value
	export let warnRange = [2.5, 7.5];
	export let nominalRange = [3.5, 6.5];
	export let value = 5;

	let scaleValue = (value, range) => {
		return ((value - range[0]) / (range[1] - range[0])) * 10;
	};

	let warnWidth = scaleValue(warnRange[1] - warnRange[0], [0, fullRange[1] - fullRange[0]]);
	let warnStart = scaleValue(warnRange[0], fullRange);

	let nominalWidth = scaleValue(nominalRange[1] - nominalRange[0], [
		0,
		fullRange[1] - fullRange[0]
	]);
	let nominalStart = scaleValue(nominalRange[0], fullRange);

	let position = Math.min(Math.max(scaleValue(value, fullRange), .1), 9.9);
</script>

<div class="w-40 h-6 bg-red-600 rounded-lg relative shadow-lg overflow-clip">
	<div
		class="h-6 bg-orange-500 absolute shadow-lg rounded-lg"
		style="width: {warnWidth}rem; left: {warnStart}rem;"
	/>
	<div
		class="h-6 bg-green-500 rounded-lg absolute shadow-lg"
		style="width: {nominalWidth}rem; left: {nominalStart}rem;"
	/>
	<div class="h-6 bg-slate-700 absolute shadow-lg" style="width: .5px; left: {position}rem;" />
</div>
