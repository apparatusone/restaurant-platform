<script lang="ts">
    import type { TableWithSession, TableStatus } from '$lib/types/index.js';

    interface Props {
        table: TableWithSession;
        status: TableStatus;
        onStatusChange?: () => void;
    }

    let { table, status, onStatusChange }: Props = $props();

    const statusColors: Record<TableStatus, string> = {
        available: 'bg-green-100 border-green-300 text-green-800',
        occupied: 'bg-red-100 border-red-300 text-red-800',
        reserved: 'bg-yellow-100 border-yellow-300 text-yellow-800'
    };

    const statusLabels: Record<TableStatus, string> = {
        available: 'Available',
        occupied: 'Occupied',
        reserved: 'Reserved'
    };

    function handleClick() {
        if (onStatusChange) {
            onStatusChange();
        }
    }
</script>

<div 
    class="w-32 rounded-lg border-2 p-4 cursor-pointer transition-all duration-200 touch-manipulation min-h-[100px] flex flex-col justify-between {statusColors[status]}"
    role="button"
    tabindex="0"
    onclick={handleClick}
    onkeydown={(e) => e.key === 'Enter' && handleClick()}
>
    <div class="text-center">
        <h3 class="font-bold text-lg mb-1">{table.code}</h3>
        <p class="text-sm opacity-75">Seats: {table.capacity}</p>
        {#if table.section}
            <p class="text-xs opacity-60 mt-1">{table.section}</p>
        {/if}
    </div>
    
    <div class="text-center mt-2">
        <span class="inline-block px-2 py-1 rounded-full text-xs font-medium bg-white bg-opacity-50">
            {statusLabels[status]}
        </span>
    </div>
</div>
