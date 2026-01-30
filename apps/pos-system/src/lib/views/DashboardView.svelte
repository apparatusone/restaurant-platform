<script lang="ts">
    import TopBar from "$lib/components/layout/TopBar.svelte";
    import BottomNav from "$lib/components/layout/BottomNav.svelte";
    import { authState } from '$lib/stores/auth.svelte';
    import { navigationStore } from '$lib/stores/navigation.svelte.js';

    import type { Component } from 'svelte';

    const user = $derived(authState.currentUser);
    const activeSection = $derived(navigationStore.activeSection);

    async function loadView(key: string): Promise<Component<any> | null> {
        switch (key) {
            case 'tables':
                return (await import('$lib/components/domain/tables/TableSection.svelte')).default;
            case 'floor':
                return (await import('$lib/components/domain/floor/Floor.svelte')).default;
            case 'kitchen':
                return (await import('$lib/components/domain/kitchen/Kitchen.svelte')).default;
            case 'inventory':
                return (await import('$lib/components/domain/inventory/Inventory.svelte')).default;
            default:
                return null;
        }
    }
</script>

<main class="h-screen bg-gray-50 flex flex-col" aria-label="Dashboard">
    {#if user}
        <TopBar user={user} />
    <section class="flex flex-1 min-h-0 overflow-hidden">
            {#await loadView(activeSection)}
                <div class="p-8 text-gray-500 text-center">Loading…</div>
            {:then View}
                {#key activeSection}
                    {#if View}
                        <View user={user} />
                    {:else}
                        <!-- Placeholder view -->
                        <div class="bg-white p-8 rounded-lg shadow-sm text-center">
                            <h2 class="m-0 mb-4 text-gray-700">{activeSection.charAt(0).toUpperCase() + activeSection.slice(1)}</h2>
                            <p class="text-gray-500 m-0">This section will be implemented soon</p>
                        </div>
                    {/if}
                {/key}
            {:catch err}
                <div class="bg-red-50 text-red-700 p-4 rounded">Failed to load section: {String(err)}</div>
            {/await}
        </section>
        <BottomNav
            fixed={false}
            activeSection={navigationStore.activeSection}
            onSectionChange={(section) => navigationStore.setActiveSection(section)}
        />
    {:else}
        <div class="p-8 text-center text-gray-500">No user loaded</div>
    {/if}
</main>

<style></style>
