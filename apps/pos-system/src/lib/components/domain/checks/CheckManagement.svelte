<script lang="ts">
    import { checksApi } from "$lib/api/endpoints/checks.js";
    import { menuAPI } from "$lib/api/endpoints/menu.js";
    import { checkStore } from "$lib/stores/check.svelte.js";
    import Accordion from "$lib/components/ui/Accordion.svelte";

    import type {
        Check,
        TableWithSession,
        TableSeating,
        CheckItem,
        MenuItem,
    } from "$lib/types/index.js";

    interface Props {
        table: TableWithSession;
        session?: TableSeating | null;
    }

    let { table, session = null }: Props = $props();

    let menuItems: MenuItem[] = $state([]); // Store menu items for lookup

    // Get check items for a check - directly from store
    function getCheckItemsForCheck(checkId: number): CheckItem[] {
        return checkStore.checkItemsByCheck.get(checkId) ?? [];
    }

    // Helper function to get menu item name by ID
    function getMenuItemName(menuItemId: number): string {
        const menuItem = menuItems.find((item) => item.id === menuItemId);
        return menuItem?.name || "Unknown Item";
    }

    async function loadMenuItems() {
        try {
            const items = await menuAPI.getMenuItems();
            menuItems = items ?? [];
        } catch (err) {
            console.error("Error loading menu items:", err);
        }
    }

    // Load menu items once
    $effect(() => {
        loadMenuItems();
    });

    // Load checks when session changes
    $effect(() => {
        if (session?.id) {
            checkStore.loadChecksForSeating(session.id);
        }
    });

    // derived state for display
    const hasActiveSession = $derived(!!session);
    const hasChecks = $derived(checkStore.checks.length > 0);
    const allChecksClosed = $derived(
        hasChecks &&
            checkStore.checks.every(
                (check: Check) => check.status === "closed",
            ),
    );
    const canFreeTable = $derived(hasActiveSession && allChecksClosed);

    // Convert checks to accordion items - derived from store

    let accordionItems = $derived(
        checkStore.checks.map((check: Check) => {
            // Calculate total amount from check items - use the helper function
            const items = getCheckItemsForCheck(check.id);
            const totalAmount = items.reduce(
                (sum, item) => sum + (Number(item.total_price) || Number(item.unit_price) * item.quantity),
                0,
            );
            const canDeleteCheck = checkStore.canDelete(check.id);

            const isPaidOrClosed = check.status === 'paid' || check.status === 'closed';
            const displayAmount = isPaidOrClosed ? 'Paid' : `$${totalAmount.toFixed(2)}`;
            
            return {
                id: check.id,
                title: `Check #${check.id} - ${displayAmount}`,
                canDelete: canDeleteCheck,
                check: check,
                isPaidOrClosed: isPaidOrClosed,
            };
        }),
    );

    function handleCheckActivate(checkId: string | number) {
        const numericCheckId = checkId as number;
        checkStore.selectCheck(numericCheckId);
    }

</script>

<div class="space-y-4">
    {#if !hasActiveSession}
        <div class="no-session flex items-center justify-center py-4">
            Reassign table
        </div>
    {:else}
        <div>
            <div class="flex justify-between items-center space-y-1">
                <h3 class="text-lg font-semibold text-gray-900">
                    Table {table.code}
                </h3>
                <span class="text-sm text-gray-600 block">
                    Started {session
                        ? new Date(session.opened_at).toLocaleTimeString(
                              "en-US",
                              {
                                  hour: "numeric",
                                  minute: "2-digit",
                                  hour12: true,
                              },
                          )
                        : ""}
                </span>
            </div>

            {#if session?.customer_name}
                <div
                    class="customer-info flex items-center gap-2 text-sm text-gray-700"
                >
                    <svg
                        class="w-4 h-4 text-gray-400"
                        fill="currentColor"
                        viewBox="0 0 20 20"
                    >
                        <path
                            fill-rule="evenodd"
                            d="M10 9a3 3 0 100-6 3 3 0 000 6zm-7 9a7 7 0 1114 0H3z"
                            clip-rule="evenodd"
                        ></path>
                    </svg>
                    <span>{session.customer_name}</span>
                </div>
            {/if}
        </div>

        <!-- Checks Accordion -->
        <Accordion
            items={accordionItems}
            activeItem={checkStore.currentCheckId}
            onItemActivate={handleCheckActivate}
            allowMultiple={false}
        >
            {#snippet children({ item })}
                {@const checkItems = getCheckItemsForCheck(item.check.id)}

                {#if checkItems.length > 0}
                    <div class="space-y-1">
                        {#each checkItems as checkItem}
                            {#each Array(checkItem.quantity) as _, index}
                                <div
                                    class="flex justify-between items-center text-sm bg-white p-2 rounded {checkItem.status ===
                                        'preparing' || 'ready'
                                        ? 'text-gray-500'
                                        : ''}"
                                >
                                    <div
                                        class={checkItem.status === "preparing"
                                            ? "opacity-60"
                                            : ""}
                                    >
                                        <span class="text-lg font-mono"
                                            >{getMenuItemName(
                                                checkItem.menu_item_id,
                                            )}</span
                                        >
                                        {#if checkItem.special_instructions}
                                            <div
                                                class="text-xs text-gray-600 italic"
                                            >
                                                {checkItem.special_instructions}
                                            </div>
                                        {/if}
                                    </div>
                                    <div class="flex items-center gap-4">
                                        <div class="text-right">
                                            <div class="font-medium">
                                                ${Number(
                                                    checkItem.unit_price,
                                                ).toFixed(2)}
                                            </div>
                                            <div
                                                class="text-xs text-gray-500 capitalize"
                                            >
                                                {checkItem.status || "pending"}
                                            </div>
                                        </div>
                                        {#if checkStore.canDeleteMenuItem(checkItem.id)}
                                            <button
                                                class="text-2xl bg-red-500 hover:bg-red-600 text-white w-10 h-10 flex items-center justify-center rounded self-stretch"
                                                onclick={async () => {
                                                    try {
                                                        await checkStore.deleteMenuItem(
                                                            checkItem.id,
                                                        );
                                                    } catch (error) {
                                                        console.error(
                                                            "Error deleting check item:",
                                                            error,
                                                        );
                                                    }
                                                }}
                                                aria-label="Delete item"
                                            >
                                                ✕
                                            </button>
                                        {/if}
                                    </div>
                                </div>
                            {/each}
                        {/each}
                    </div>
                {:else}
                    <div class="text-sm text-gray-500 italic">
                        No items in this check
                    </div>
                {/if}
            {/snippet}
        </Accordion>

        {#if canFreeTable}
            <div class="free-table-section pt-4 border-t border-gray-200">
                <div class="free-table-content text-center space-y-3">
                    <p class="text-sm text-gray-600">
                        All checks are closed. Click the Close button to free
                        this table.
                    </p>
                </div>
            </div>
        {/if}
    {/if}
</div>

<style></style>
