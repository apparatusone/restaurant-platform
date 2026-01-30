<script lang="ts">
    import MenuGrid from '$lib/components/domain/menu/MenuGrid.svelte';

    const tabs = ['Main', 'Dessert', 'Drinks'] as const;
    type Tab = typeof tabs[number];

    const { initialTab = 'Main' } = $props();
    let active = $state<Tab>(initialTab as Tab);

    let tabButtons: HTMLButtonElement[] = [];

    const baseBtn = 'px-3 py-2 rounded-md text-sm font-medium border transition-colors focus:outline-none focus:ring-2 focus:ring-indigo-500';
    const tabClass = (t: Tab) =>
        `${baseBtn} ${active === t ? 'bg-gray-900 text-white border-gray-900' : 'text-gray-600 border-transparent hover:bg-gray-100 hover:text-gray-800'}`;

    function setActive(t: Tab) {
        active = t;
    }

    function onKeydown(e: KeyboardEvent, idx: number) {
        const key = e.key;
        let next = idx;
        if (key === 'ArrowRight') next = (idx + 1) % tabs.length;
        else if (key === 'ArrowLeft') next = (idx - 1 + tabs.length) % tabs.length;
        else if (key === 'Home') next = 0;
        else if (key === 'End') next = tabs.length - 1;
        else return;
        e.preventDefault();
        active = tabs[next];
        tabButtons[next]?.focus();
    }
</script>

<section class="w-full h-full flex flex-col">
    <!-- header which holds buttons (tabs) main, dessert, drinks -->
    <div role="tablist" aria-label="Menu categories" class="flex items-center gap-2 border-b bg-white p-2 sticky top-0 z-10">
        {#each tabs as t, i}
            <button
                bind:this={tabButtons[i]}
                id={`tab-${t.toLowerCase()}`}
                role="tab"
                aria-selected={active === t}
                aria-controls={`panel-${t.toLowerCase()}`}
                tabindex={active === t ? 0 : -1}
                class={tabClass(t)}
                onclick={() => setActive(t)}
                onkeydown={(e) => onKeydown(e, i)}
            >
                {t}
            </button>
        {/each}
    </div>

    <!-- section which holds the sections tabs opens  -->
    <div class="flex-1 overflow-auto bg-gray-50">
        <div 
            id="panel-{active.toLowerCase()}" 
            role="tabpanel" 
            aria-labelledby="tab-{active.toLowerCase()}" 
            class="h-full"
        >
            {#if active === 'Main'}
                <MenuGrid />
            {:else}
                <div class="flex items-center justify-center h-32">
                    <div class="text-gray-500">No {active.toLowerCase()} items available</div>
                </div>
            {/if}
        </div>
    </div>
</section>

<style></style>