<script lang="ts">
	import { page } from '$app/stores';
	import {
		Sidebar,
		SidebarGroup,
		SidebarItem,
		SidebarWrapper,
		SidebarDropdownItem,
		SidebarDropdownWrapper
	} from 'flowbite-svelte';
	import {
		MapPinAltSolid,
		UserGroupSolid,
		AdjustmentsHorizontalSolid,
		BookSolid
	} from 'flowbite-svelte-icons';
	$: activeUrl = $page.url.pathname;
</script>

<Sidebar {activeUrl}>
	<SidebarWrapper>
		<SidebarGroup>
			<SidebarItem label="Biometrics" href="/biometrics">
				<svelte:fragment slot="icon">
					<AdjustmentsHorizontalSolid
						class="w-5 h-5 text-gray-500 transition duration-75 dark:text-gray-400 group-hover:text-gray-900 dark:group-hover:text-white"
					/>
				</svelte:fragment>
			</SidebarItem>
			<SidebarDropdownWrapper label="Camera Feeds" isOpen={true}>
				<svelte:fragment slot="icon">
					<UserGroupSolid
						class="w-5 h-5 text-gray-500 transition duration-75 dark:text-gray-400 group-hover:text-gray-900 dark:group-hover:text-white"
					/>
				</svelte:fragment>
				<SidebarDropdownItem label="Rover" href="/rover" active={activeUrl == '/rover'} />
				<SidebarDropdownItem
					label="Astroanut"
					href="/astronaut"
					active={activeUrl == '/astronaut'}
				/>
			</SidebarDropdownWrapper>
			<SidebarItem label="Map" href="/map">
				<svelte:fragment slot="icon">
					<MapPinAltSolid
						class="w-5 h-5 text-gray-500 transition duration-75 dark:text-gray-400 group-hover:text-gray-900 dark:group-hover:text-white"
					/>
				</svelte:fragment>
			</SidebarItem>
			<SidebarItem label="Procedures" href="/procedures" active={activeUrl == '/procedures'}>
				<svelte:fragment slot="icon">
					<BookSolid
						class="w-5 h-5 text-gray-500 transition duration-75 dark:text-gray-400 group-hover:text-gray-900 dark:group-hover:text-white"
					/>
				</svelte:fragment>
			</SidebarItem>
		</SidebarGroup>
	</SidebarWrapper>
</Sidebar>
