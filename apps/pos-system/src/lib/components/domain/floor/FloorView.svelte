<script lang="ts">
    import type { TableWithSession, TableStatus } from '$lib/types/index.js';
    import TableComponent from '$lib/components/domain/floor/Table.svelte';

    interface Props {
        loading: boolean;
        error: string;
        tablesBySection: Record<string, TableWithSession[]>;
        selectedTable?: TableWithSession;
        getTableStatus: (table: TableWithSession) => TableStatus;
        onTableSelect: (table: TableWithSession) => void;
        onRefresh: () => void;
    }

    let { loading, error, tablesBySection, selectedTable, getTableStatus, onTableSelect, onRefresh }: Props = $props();
</script>

<section class="w-full h-full flex flex-col px-4">
    <!-- Status Legend -->
    <div class="flex justify-between items-center">
        <div class="flex flex-wrap gap-4 mb-6 p-3 bg-gray-50 rounded-lg">
            <div class="flex items-center gap-2">
                <div class="w-4 h-4 rounded bg-green-100 border border-green-300"></div>
                <span class="text-sm text-gray-700">Available</span>
            </div>
            <div class="flex items-center gap-2">
                <div class="w-4 h-4 rounded bg-yellow-100 border border-yellow-300"></div>
                <span class="text-sm text-gray-700">Reserved</span>
            </div>
            <div class="flex items-center gap-2">
                <div class="w-4 h-4 rounded bg-red-100 border border-red-300"></div>
                <span class="text-sm text-gray-700">Occupied</span>
            </div>
            <div class="text-sm text-gray-500 ml-4">
                Tap tables to view details
            </div>
        </div>

        <button 
            class="w-20 h-10 bg-blue-600 text-white rounded-lg transition-colors"
            onclick={onRefresh}
        >
            Refresh
        </button>
    </div>

    {#if loading}
        <div class="flex justify-center items-center py-8">
            <div class="animate-spin rounded-full h-8 w-8 border-b-2 border-blue-600"></div>
            <span class="ml-2 text-gray-600">Loading tables...</span>
        </div>
    {:else if error}
        <div class="bg-red-50 border border-red-200 rounded-lg p-4 mb-4">
            <p class="text-red-800">Error: {error}</p>
            <button 
                class="mt-2 px-3 py-1 bg-red-600 text-white rounded hover:bg-red-700 transition-colors text-sm"
                onclick={onRefresh}
            >
                Try Again
            </button>
        </div>
    {:else if Object.keys(tablesBySection).length === 0}
        <div class="text-center py-8 text-gray-500">
            <p>No tables found</p>
        </div>
    {:else}
        {#each Object.entries(tablesBySection) as [section, sectionTables]}
            <div class="mb-8">
                <h2 class="text-lg font-semibold text-gray-800 mb-4">{section}</h2>
                <div class="flex flex-wrap gap-4">
                    {#each sectionTables as table}
                        <div class="relative">
                            <TableComponent 
                                {table}
                                status={getTableStatus(table)}
                                onStatusChange={() => onTableSelect(table)}
                            />
                            {#if selectedTable?.id === table.id}
                                <div class="absolute inset-0 border-2 border-blue-500 rounded-lg pointer-events-none"></div>
                            {/if}
                        </div>
                    {/each}
                </div>
            </div>
        {/each}
    {/if}
</section>

<style></style>