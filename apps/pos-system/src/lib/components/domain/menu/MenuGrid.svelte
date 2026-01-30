<script lang="ts">
    import { onMount } from 'svelte';
    import MenuCard from './MenuCard.svelte';
    import { menuAPI } from '$lib/api/endpoints/menu.js';
    import type { MenuItem as MenuItemType } from '$lib/types/index.js';

    let menuItems: MenuItemType[] = $state([]);
    let loading = $state(false);
    let error = $state<string | null>(null);

    // load menu items on mount
    onMount(async () => {
        try {
            loading = true;
            error = null;
            const items = await menuAPI.getMenuItems();
            menuItems = items;
        } catch (err) {
            console.error('Failed to load menu:', err);
            error = 'Failed to load menu items';
        } finally {
            loading = false;
        }
    });

</script>

<div class="h-full flex flex-col">
    {#if loading}
        <div class="flex items-center justify-center h-32">
            <div class="text-gray-500">Loading menu...</div>
        </div>
    {:else if error}
        <div class="flex items-center justify-center h-32">
            <div class="text-red-500">Error: {error}</div>
        </div>
    {:else if menuItems.length === 0}
        <div class="flex items-center justify-center h-32">
            <div class="text-gray-500">No menu items available</div>
        </div>
    {:else}
        <div class="grid grid-cols-1 sm:grid-cols-2 lg:grid-cols-3 gap-4 p-4">
            {#each menuItems as item (item.id)}
                <MenuCard {item}/>
            {/each}
        </div>
    {/if}
</div>