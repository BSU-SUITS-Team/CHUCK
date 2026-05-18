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
</script>

<nav class="w-64 h-full overflow-y-auto p-4" aria-label="Main navigation">
	<ul class="space-y-2">
		<li>
			<a href="/biometrics" class="{baseLink} {activeUrl === '/biometrics' ? activeLink : inactiveLink}">
				<AdjustmentsHorizontalSolid class={iconClass} />
				<span class="ms-3">Biometrics</span>
			</a>
		</li>
		<li>
			<div class="{baseLink} {inactiveLink}">
				<UsersGroupSolid class={iconClass} />
				<span class="ms-3 flex-1">Camera Feeds</span>
			</div>
			<ul class="mt-2 space-y-2 py-2 pl-8">
				<li>
					<a href="/rover" class="{baseLink} {activeUrl === '/rover' ? activeLink : inactiveLink}">Rover</a>
				</li>
				<li>
					<a href="/astronaut" class="{baseLink} {activeUrl === '/astronaut' ? activeLink : inactiveLink}">Astronaut</a>
				</li>
			</ul>
		</li>
		<li>
			<a href="/map" class="{baseLink} {activeUrl === '/map' ? activeLink : inactiveLink}">
				<MapPinAltSolid class={iconClass} />
				<span class="ms-3">Map</span>
			</a>
		</li>
		<li>
			<a href="/events" class="{baseLink} {activeUrl === '/events' ? activeLink : inactiveLink}">
				<AnnotationOutline class={iconClass} />
				<span class="ms-3">Event Log</span>
			</a>
		</li>
		<li>
			<a href="/windows" class="{baseLink} {activeUrl === '/windows' ? activeLink : inactiveLink}">
				<WindowOutline class={iconClass} />
				<span class="ms-3">Windows</span>
			</a>
		</li>
		<li>
			<a href="/procedures" class="{baseLink} {activeUrl === '/procedures' ? activeLink : inactiveLink}">
				<BookSolid class={iconClass} />
				<span class="ms-3">Procedures</span>
			</a>
		</li>
	</ul>
</nav>
