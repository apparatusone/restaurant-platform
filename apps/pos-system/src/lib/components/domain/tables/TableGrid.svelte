<script lang="ts">
    import type {
        TableSeating,
        TableStatus,
        TableWithSession,
    } from "$lib/types/index.js";

    interface Props {
        user?: {
            id: string;
            name: string;
            role: string;
        };
        tables: TableWithSession[];
        sessions: TableSeating[];
        selectedTable: TableWithSession | null;
        loading: boolean;
        error: string;
        onSelectTable: (table: TableWithSession) => void;
    }

    let {
        user,
        tables,
        sessions,
        selectedTable,
        loading,
        error,
        onSelectTable,
    }: Props = $props();

    const serverTables = $derived.by(() => {
        if (user?.role === "server") {
            return tables.filter((table) => {
                const session = sessions.find(
                    (s) => s.id === table.current_seating_id,
                );
                return (
                    session &&
                    session.assigned_server_id === parseInt(user.id)
                );
            });
        }
        return tables.filter((table) => table.current_seating_id);
    });

    const groupedTables = $derived.by(() => {
        const groups = new Map<number, TableWithSession[]>();
        const ungroupedTables: TableWithSession[] = [];

        serverTables.forEach((table) => {
            if (table.current_seating_id) {
                const seatingId = table.current_seating_id;
                if (!groups.has(seatingId)) {
                    groups.set(seatingId, []);
                }
                groups.get(seatingId)!.push(table);
            } else {
                ungroupedTables.push(table);
            }
        });

        const result: {
            type: "single" | "group";
            tables: TableWithSession[];
            seatingId?: number;
        }[] = [];

        groups.forEach((tablesInGroup, seatingId) => {
            if (tablesInGroup.length > 1) {
                result.push({ type: "group", tables: tablesInGroup, seatingId });
            } else {
                result.push({ type: "single", tables: tablesInGroup });
            }
        });

        ungroupedTables.forEach((table) => {
            result.push({ type: "single", tables: [table] });
        });

        return result;
    });

    function getTableStatus(table: TableWithSession): TableStatus {
        if (!table.current_seating_id) return "available";
        const session = sessions.find((s) => s.id === table.current_seating_id);
        if (!session || session.closed_at) return "available";
        return session.is_reservation ? "reserved" : "occupied";
    }
</script>

<section class="w-full h-full flex flex-col min-h-0">
    <div class="text-xl flex border-b border-gray-200 bg-white px-4 py-2">
        <div class="font-medium text-gray-900">Tables</div>
    </div>

    <div class="flex-1 min-h-0 p-2 overflow-y-auto">
        {#if loading}
            <div class="flex items-center justify-center h-full">
                <div class="text-gray-500">Loading tables...</div>
            </div>
        {:else if error}
            <div class="flex items-center justify-center h-full">
                <div class="text-red-500">Error: {error}</div>
            </div>
        {:else}
            <div class="grid grid-cols-2 2xl:grid-cols-3 gap-4">
                {#each groupedTables as item}
                    {#each item.tables as table}
                        {@const status = getTableStatus(table)}
                        <button
                            onclick={() => onSelectTable(table)}
                            class={`p-4 border-2 rounded-lg text-left hover:opacity-90 transition-colors ${selectedTable?.id === table.id ? "border-blue-500" : "border-gray-200"}`}
                        >
                            <div class="flex items-center justify-between mb-2">
                                <h3 class="font-bold text-lg">{table.code}</h3>
                                <div
                                    class="w-3 h-3 rounded-full"
                                    class:bg-green-500={status === "available"}
                                    class:bg-red-500={status === "occupied"}
                                    class:bg-yellow-500={status === "reserved"}
                                ></div>
                            </div>
                            {#if table.section}
                                <p class="text-xs opacity-60 mt-1">
                                    {table.section}
                                </p>
                            {/if}
                        </button>
                    {/each}
                {/each}
            </div>

            {#if groupedTables.length === 0}
                <div class="text-center text-gray-500 mt-8">
                    {#if user?.role === "server"}
                        No tables assigned to you
                    {:else}
                        No tables available
                    {/if}
                </div>
            {/if}
        {/if}
    </div>
</section>

<style></style>
