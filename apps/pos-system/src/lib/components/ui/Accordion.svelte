<script lang="ts">
    import type { Snippet } from 'svelte';

    interface AccordionItem {
        id: string | number;
        title: string;
        content?: any;
        [key: string]: any; // Allow additional properties
    }

    interface Props {
        items: AccordionItem[];
        activeItem?: string | number | null;
        onItemToggle?: (itemId: string | number) => void;
        onItemActivate?: (itemId: string | number) => void;
        allowMultiple?: boolean;
        children?: Snippet<[{ item: AccordionItem }]>;
        empty?: Snippet;
    }

    let { 
        items, 
        activeItem = null,
        onItemToggle,
        onItemActivate,
        allowMultiple = false,
        children,
        empty
    }: Props = $props();

    let expandedItems = $state<Set<string | number>>(new Set());

    function toggleItem(itemId: string | number) {
        if (allowMultiple) {
            if (expandedItems.has(itemId)) {
                expandedItems.delete(itemId);
            } else {
                expandedItems.add(itemId);
            }
        } else {
            // Single expand mode
            if (expandedItems.has(itemId)) {
                expandedItems.clear();
            } else {
                expandedItems.clear();
                expandedItems.add(itemId);
            }
        }
        
        expandedItems = new Set(expandedItems);
        
        onItemToggle?.(itemId);

        // If the item is now expanded, treat that as activation
        if (isExpanded(itemId)) {
            onItemActivate?.(itemId);
        }
    }

    function isExpanded(itemId: string | number): boolean {
        return expandedItems.has(itemId);
    }

    function isActive(itemId: string | number): boolean {
        return activeItem === itemId;
    }
</script>

<div class="space-y-0">
    {#each items as item (item.id)}
        <div 
            class="border-b border-gray-200 {isActive(item.id) ? 'border-l-4 border-l-blue-500' : ''} {item.isPaidOrClosed ? 'opacity-60 bg-gray-50' : ''}"
        >
            <button 
                class="flex justify-between items-center p-3 w-full border-none bg-transparent text-left cursor-pointer select-none hover:bg-gray-50 {isExpanded(item.id) ? 'bg-blue-50' : ''}"
                onclick={() => toggleItem(item.id)}
                type="button"
            >
                <div class="flex-1 font-medium {item.isPaidOrClosed ? 'text-gray-500' : ''}">
                    {item.title}
                </div>
                <div class="flex items-center gap-2">
                    <div class="text-xs transition-transform duration-200 {isExpanded(item.id) ? 'rotate-180' : ''}">
                        ▼
                    </div>
                </div>
            </button>
            
            {#if isExpanded(item.id)}
                <div class="p-4 border-t border-gray-100 bg-gray-50">
                    {#if item.content}
                        {@render item.content(item)}
                    {:else if children}
                        {@render children({ item })}
                    {:else}
                        <p>No content provided for {item.title}</p>
                    {/if}
                </div>
            {/if}
        </div>
    {/each}
    
    {#if items.length === 0}
        <div class="p-6 text-center text-gray-600">
            {#if empty}
                {@render empty()}
            {:else}
                <p>No items to display</p>
            {/if}
        </div>
    {/if}
</div>
