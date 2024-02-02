/** @type {import('./$types').PageLoad} */
export async function load({ params }) {
	return {
		name: params.procedureName
	};
}
