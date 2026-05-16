<script lang="ts">
	import { page } from '$app/stores';
	import {
		MapPinAltSolid,
		UsersGroupSolid,
		AdjustmentsHorizontalSolid,
		BookSolid,
		AnnotationOutline,
		WindowOutline
	} from 'flowbite-svelte-icons';

	$: activeUrl = $page.url.pathname;

	const baseLink =
		'group flex items-center rounded-lg p-2 text-base font-normal transition-colors';
	const activeLink = 'bg-gray-100 text-gray-900 dark:bg-gray-700 dark:text-white';
	const inactiveLink =
		'text-gray-900 hover:bg-gray-100 dark:text-white dark:hover:bg-gray-700';
	const iconClass =
		'w-5 h-5 text-gray-500 transition duration-75 dark:text-gray-400 group-hover:text-gray-900 dark:group-hover:text-white';

	const navClass = (href: string) =>
		`${baseLink} ${activeUrl === href ? activeLink : inactiveLink}`;
</script>

<nav class="w-64 h-full overflow-y-auto p-4" aria-label="Main navigation">
	<ul class="space-y-2">
		<li>
			<a href="/biometrics" class={navClass('/biometrics')}>
				<AdjustmentsHorizontalSolid class={iconClass} />
				<span class="ms-3">Biometrics</span>
			</a>
		</li>
		<li>
			<div class={`${baseLink} ${inactiveLink}`}>
				<UsersGroupSolid class={iconClass} />
				<span class="ms-3 flex-1">Camera Feeds</span>
			</div>
			<ul class="mt-2 space-y-2 py-2 pl-8">
				<li>
					<a href="/rover" class={navClass('/rover')}>Rover</a>
				</li>
				<li>
					<a href="/astronaut" class={navClass('/astronaut')}>Astronaut</a>
				</li>
			</ul>
		</li>
		<li>
			<a href="/map" class={navClass('/map')}>
				<MapPinAltSolid class={iconClass} />
				<span class="ms-3">Map</span>
			</a>
		</li>
		<li>
			<a href="/events" class={navClass('/events')}>
				<AnnotationOutline class={iconClass} />
				<span class="ms-3">Event Log</span>
			</a>
		</li>
		<li>
			<a href="/windows" class={navClass('/windows')}>
				<WindowOutline class={iconClass} />
				<span class="ms-3">Windows</span>
			</a>
		</li>
		<li>
			<a href="/procedures" class={navClass('/procedures')}>
				<BookSolid class={iconClass} />
				<span class="ms-3">Procedures</span>
			</a>
		</li>
	</ul>
</nav>
