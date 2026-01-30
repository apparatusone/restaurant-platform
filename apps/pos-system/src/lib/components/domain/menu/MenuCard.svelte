<script lang="ts">
    import type { MenuItem } from '$lib/types/index.js';
    import { checkStore } from '$lib/stores/check.svelte.js';

    interface Props {
        item: MenuItem;
    }

    let { item }: Props = $props();

    const cardClasses = "bg-white rounded-lg border border-gray-200 p-4 hover:shadow-md transition-shadow duration-200";
    const nameClasses = "font-semibold text-gray-900 mb-1 break-words whitespace-normal";
    
    function getCategoryColor(category: string): string {
        switch (category) {
            case 'vegetarian':
                return 'bg-green-100 text-green-800';
            case 'vegan':
                return 'bg-emerald-100 text-emerald-800';
            case 'gluten_free':
                return 'bg-blue-100 text-blue-800';
            default:
                return 'bg-gray-100 text-gray-800';
        }
    }

    function formatCategory(category: string): string {
        return category.replace('_', ' ').replace(/\b\w/g, l => l.toUpperCase());
    }

    async function handleAddToCheck() {
        const currentCheck = checkStore.currentCheck;
        if (!currentCheck) {
            console.log("No check selected");
            return;
        }

        // Prevent adding items to paid or closed checks
        if (currentCheck.status === 'paid' || currentCheck.status === 'closed') {
            alert(`Cannot add items to a ${currentCheck.status} check.`);
            return;
        }

        try {
            await checkStore.addOrderItem({
                check_id: currentCheck.id,
                menu_item_id: item.id,
                quantity: 1
            });
        } catch (error) {
            console.error('Failed to add item to check:', error);
            alert('Failed to add item to check. Please try again.');
        }
    }

    // Disable button if check is paid or closed
    const isCheckPaidOrClosed = $derived(
        checkStore.currentCheck?.status === 'paid' || 
        checkStore.currentCheck?.status === 'closed'
    );
</script>

<button 
    class="{cardClasses} {isCheckPaidOrClosed ? 'opacity-50 cursor-not-allowed' : 'cursor-pointer'}"
    onclick={handleAddToCheck}
    disabled={isCheckPaidOrClosed}
>
    <div class="flex justify-center">
        <h3 class={nameClasses}>{item.name}</h3>
    </div>
    
    <div class="mt-2 flex items-center justify-between">
        {#if item.food_category !== 'regular'}
            <span class="inline-flex items-center px-2 py-1 rounded-full text-xs font-medium {getCategoryColor(item.food_category)}">
                {formatCategory(item.food_category)}
            </span>
        {/if}
    </div>
</button>