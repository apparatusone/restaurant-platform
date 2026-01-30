<script lang="ts">
    import { onMount } from 'svelte';
    import { checkItemsApi } from '$lib/api/endpoints/check-items.js';
    import { menuAPI } from '$lib/api/endpoints/menu.js';
    import type { CheckItem } from '$lib/types/restaurant.js';

    let preparingItems = $state<CheckItem[]>([]);
    let loading = $state(false);
    let error = $state<string | null>(null);
    let menuItems = $state<{[key: number]: any}>({});

    // load preparing items on mount
    onMount(() => {
        loadPreparingItems();
        // refresh every 30 seconds
        const interval = setInterval(loadPreparingItems, 30000);
        return () => clearInterval(interval);
    });

    async function loadPreparingItems() {
        try {
            loading = true;
            error = null;
            preparingItems = await checkItemsApi.getAllCheckItems('preparing');
            
            // load menu item details for each item
            const menuItemIds = [...new Set(preparingItems.map(item => item.menu_item_id))];
            for (const menuItemId of menuItemIds) {
                if (!menuItems[menuItemId]) {
                    try {
                        menuItems[menuItemId] = await menuAPI.getMenuItem(menuItemId);
                    } catch (err) {
                        console.error(`Error loading menu item ${menuItemId}:`, err);
                    }
                }
            }
        } catch (err) {
            error = 'Failed to load kitchen items';
            console.error('Error loading preparing items:', err);
        } finally {
            loading = false;
        }
    }

    async function markItemReady(itemId: number) {
        try {
            await checkItemsApi.markItemReady(itemId);
            // refresh the list
            await loadPreparingItems();
        } catch (err) {
            error = 'Failed to mark item as ready';
            console.error('Error marking item ready:', err);
        }
    }
</script>

<div class="p-4 h-full bg-gray-50">
    <div class="flex justify-between items-center mb-6 pb-4 border-b-2 border-gray-200">
        <h1 class="m-0 text-gray-700 text-2xl">Kitchen View</h1>
        <button 
            onclick={loadPreparingItems} 
            class="px-4 py-2 bg-blue-500 text-white border-none rounded cursor-pointer text-sm hover:bg-blue-600 disabled:bg-gray-500 disabled:cursor-not-allowed" 
            disabled={loading}
        >
            {loading ? 'Loading...' : 'Refresh'}
        </button>
    </div>

    {#if error}
        <div class="bg-red-100 text-red-800 px-3 py-2 rounded mb-4 border border-red-200">
            {error}
        </div>
    {/if}

    <div class="grid grid-cols-[repeat(auto-fill,minmax(300px,1fr))] gap-4">
        {#if preparingItems.length === 0}
            <div class="col-span-full text-center py-12 text-gray-500 text-lg">
                {loading ? 'Loading items...' : 'No items to prepare'}
            </div>
        {:else}
            {#each preparingItems as item (item.id)}
                <div class="bg-white rounded-lg p-4 shadow-sm border-l-4 border-yellow-400">
                    <div class="flex justify-between items-start mb-3">
                        <div>
                            <h3 class="m-0 mb-1 text-gray-700 text-lg">{menuItems[item.menu_item_id]?.name || 'Loading...'}</h3>
                            <div class="text-xs text-gray-500 font-medium">
                                Check #{item.check_id}
                            </div>
                        </div>
                        <span class="bg-gray-200 text-gray-700 px-2 py-1 rounded font-bold text-sm">×{item.quantity}</span>
                    </div>
                    
                    {#if item.special_instructions}
                        <div class="bg-yellow-50 border border-yellow-200 rounded px-2 py-2 mb-3 text-sm text-yellow-800">
                            <strong>Special:</strong> {item.special_instructions}
                        </div>
                    {/if}
                    
                    <div class="flex justify-end">
                        <button 
                            class="bg-green-500 text-white border-none px-4 py-2 rounded cursor-pointer font-medium transition-colors hover:bg-green-600 active:translate-y-px"
                            onclick={() => markItemReady(item.id)}
                        >
                            Mark Ready
                        </button>
                    </div>
                </div>
            {/each}
        {/if}
    </div>
</div>