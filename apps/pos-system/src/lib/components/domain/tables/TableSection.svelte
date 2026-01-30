<script lang="ts">
    import { onMount } from "svelte";
    import type {
        TableSeating,
        TableWithSession,
    } from "$lib/types/index.js";

    import TableGrid from "$lib/components/domain/tables/TableGrid.svelte";
    import CheckManagement from "$lib/components/domain/checks/CheckManagement.svelte";
    import Menu from "$lib/components/domain/menu/Menu.svelte";
    import PaymentSection from "$lib/components/domain/payments/PaymentSection.svelte";

    import { checkStore } from "$lib/stores/check.svelte.js";

    interface Props {
        user?: {
            id: string;
            name: string;
            role: string;
        };
    }

    let { user }: Props = $props();

    let tables = $state<TableWithSession[]>([]);
    let sessions = $state<TableSeating[]>([]);

    let selectedTable = $state<TableWithSession | null>(null);

    let loading = $state(true);
    let error = $state("");

    let showPaymentSection = $state(false);
    let showFreeTableModal = $state(false);

    const selectedSeatingId = $derived(selectedTable?.current_seating_id ?? null);
    const selectedSession = $derived(
        selectedSeatingId
            ? sessions.find((s) => s.id === selectedSeatingId) || null
            : null,
    );

    $effect(() => {
        const seatingId = selectedTable?.current_seating_id;
        if (seatingId) {
            checkStore.loadChecksForSeating(seatingId);
        } else {
            checkStore.clear();
        }
    });

    async function loadTableData() {
        try {
            loading = true;
            error = "";

            const tablesModule = await import("$lib/api/endpoints/tables.js");
            const { tablesApi, tableSeatingsApi } = tablesModule;

            const [tablesData, sessionsData] = await Promise.all([
                tablesApi.getTablesWithSessions(),
                tableSeatingsApi.getActiveSeatings(),
            ]);

            tables = tablesData;
            sessions = sessionsData;
        } catch (err) {
            error =
                err instanceof Error ? err.message : "Failed to load table data";
            console.error("Error loading table data:", err);
        } finally {
            loading = false;
        }
    }

    async function handleCloseCurrentCheck() {
        if (!checkStore.currentCheckId) return;
        try {
            const { checksApi } = await import("$lib/api/endpoints/checks.js");
            const closedCheck = await checksApi.closeCheck(
                checkStore.currentCheckId,
            );

            if (closedCheck.seating_id) {
                await checkStore.loadChecksForSeating(closedCheck.seating_id);
            }
        } catch (err) {
            console.error("Failed to close check:", err);
        }
    }

    async function handleCreateCheck() {
        if (!selectedSeatingId) return;
        try {
            const { checksApi } = await import("$lib/api/endpoints/checks.js");
            const newCheck = await checksApi.createCheck({
                seating_id: selectedSeatingId,
                is_virtual: false,
                subtotal: 0,
                tax_amount: 0,
                tip_amount: 0,
                total_amount: 0,
            });
            checkStore.addCheck(newCheck);
        } catch (err) {
            console.error("Error creating check:", err);
        }
    }

    async function handleFreeTable() {
        if (!selectedSeatingId) return;

        try {
            const { tableSeatingsApi } = await import(
                "$lib/api/endpoints/tables.js"
            );
            await tableSeatingsApi.closeSeating(selectedSeatingId);
            await loadTableData();
            selectedTable = null;
            checkStore.clear();
        } catch (err) {
            const message =
                err instanceof Error ? err.message : JSON.stringify(err);
            console.error("Failed to free table:", err);
            alert(message);
        }
    }

    function handleSelectTable(table: TableWithSession) {
        selectedTable = table;
    }

    onMount(() => {
        loadTableData();
    });
</script>

{#if showPaymentSection}
    <PaymentSection onBack={() => (showPaymentSection = false)} />
{:else}
    <div class="h-full w-full flex min-h-0">
        <section class="w-3/10 min-w-[120px] border-r border-gray-200 bg-white min-h-0">
            <TableGrid
                {user}
                {tables}
                {sessions}
                {loading}
                {error}
                selectedTable={selectedTable}
                onSelectTable={handleSelectTable}
            />
        </section>

        <section class="w-3/10 min-w-[360px] bg-gray-50 border-r border-gray-200 min-h-0 flex flex-col">
            {#if selectedTable}
                <div class="flex-1 min-h-0 overflow-y-auto p-2">
                    <CheckManagement table={selectedTable} session={selectedSession} />
                </div>

                <div class="mt-auto p-4 text-2xl">
                    <div class="grid grid-cols-3 gap-2">
                        <button
                            onclick={handleCloseCurrentCheck}
                            disabled={!checkStore.currentCheckId}
                            class="{checkStore.currentCheckId
                                ? 'bg-green-600 text-white hover:bg-green-700'
                                : 'bg-gray-300 text-gray-500'} rounded px-4 py-2 text-base"
                        >
                            Close
                        </button>

                        <button
                            onclick={async () => {
                                try {
                                    await checkStore.sendToKitchen();
                                } catch (err) {
                                    console.error('Error sending items:', err);
                                }
                            }}
                            disabled={!checkStore.canSendToKitchen}
                            class="{checkStore.canSendToKitchen
                                ? 'bg-gray-200 text-gray-700 hover:bg-gray-300'
                                : 'bg-gray-300 text-gray-500'} rounded px-4 py-2 text-base"
                        >
                            Send
                        </button>

                        <button
                            onclick={handleCreateCheck}
                            disabled={!selectedSeatingId}
                            class="{selectedSeatingId
                                ? 'bg-gray-200 text-gray-700 hover:bg-gray-300'
                                : 'bg-gray-300 text-gray-500'} rounded px-4 py-2 text-base"
                        >
                            Add Check
                        </button>

                        <button
                            disabled={true}
                            class="bg-gray-300 text-gray-500 rounded px-4 py-2 text-base"
                            aria-label="Merge Checks"
                        >
                            Merge
                        </button>

                        <button
                            onclick={() => (showPaymentSection = true)}
                            disabled={!checkStore.currentCheck}
                            class="{checkStore.currentCheck
                                ? 'bg-blue-500 text-white hover:bg-blue-600'
                                : 'bg-gray-300 text-gray-500'} rounded px-4 py-2 text-base"
                        >
                            Pay
                        </button>

                        <button
                            onclick={() => (showFreeTableModal = true)}
                            disabled={!selectedSeatingId}
                            class="{selectedSeatingId
                                ? 'bg-red-500 text-white hover:bg-red-600'
                                : 'bg-gray-300 text-gray-500'} rounded px-4 py-2 text-base"
                            aria-label="Free table"
                        >
                            More
                        </button>
                    </div>
                </div>
            {:else}
                <div class="flex-1 flex items-center justify-center p-6">
                    <p class="text-gray-500 text-center">
                        Select a table to manage checks<br />
                        <span class="text-sm">and take orders</span>
                    </p>
                </div>
            {/if}
        </section>

        <section class="flex-1 bg-white min-h-0">
            <Menu />
        </section>
    </div>

    {#if showFreeTableModal && selectedTable}
        <div class="fixed inset-0 bg-black bg-opacity-50 flex items-center justify-center z-50">
            <div class="bg-white rounded-lg p-6 max-w-md w-full mx-4">
                <h3 class="text-xl font-semibold mb-4">
                    Free Table {selectedTable.code}?
                </h3>
                <p class="text-gray-600 mb-6">
                    This will close the table seating. The table will become available for new customers.
                </p>
                <div class="flex gap-3 justify-end">
                    <button
                        onclick={() => (showFreeTableModal = false)}
                        class="px-4 py-2 border border-gray-300 rounded hover:bg-gray-50"
                    >
                        Cancel
                    </button>
                    <button
                        onclick={async () => {
                            await handleFreeTable();
                            showFreeTableModal = false;
                        }}
                        class="px-4 py-2 bg-red-500 text-white rounded hover:bg-red-600"
                    >
                        Free Table
                    </button>
                </div>
            </div>
        </div>
    {/if}
{/if}

<style></style>
