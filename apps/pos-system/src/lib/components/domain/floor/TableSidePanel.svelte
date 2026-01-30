<script lang="ts">
    import { onMount } from 'svelte';
    import type { TableWithSession, TableSeating, TableStatus, Staff, Check } from '$lib/types/index.js';

    interface Props {
        selectedTable?: TableWithSession;
        currentSeating?: TableSeating;
        staff: Staff[];
        tableStatus?: TableStatus;
        onClose: () => void;
        onCreateSeating: (tableId: number, isReservation: boolean, customerName?: string, assignedServerId?: number) => void;
        onUpdateSeating: (seatingId: number, updates: Partial<TableSeating>) => void;
        onCloseSeating: (seatingId: number) => void;
    }

    let { selectedTable, currentSeating, staff, tableStatus, onClose, onCreateSeating, onUpdateSeating, onCloseSeating }: Props = $props();

    let customerName = $state('');
    let isReservation = $state(false);
    let assignedServerId = $state<number | undefined>();
    let sessionNotes = $state('');
    let sessionChecks = $state<Check[]>([]);
    let loadingChecks = $state(false);

    function startNewSession() {
        if (!selectedTable) return;
        onCreateSeating(selectedTable.id, isReservation, isReservation ? customerName : undefined, assignedServerId);
        resetForm();
    }

    function updateSessionDetails() {
        if (!currentSeating) return;
        onUpdateSeating(currentSeating.id, {
            customer_name: customerName || undefined,
            assigned_server_id: assignedServerId,
            notes: sessionNotes || undefined
        });
    }

    function seatReservation() {
        if (!currentSeating) return;
        onUpdateSeating(currentSeating.id, {
            is_reservation: false,
            assigned_server_id: assignedServerId,
            customer_name: customerName || undefined,
            notes: sessionNotes || undefined
        });
    }

    function endSession() {
        if (!currentSeating) return;
        onCloseSeating(currentSeating.id);
    }

    function resetForm() {
        customerName = '';
        isReservation = false;
        assignedServerId = undefined;
        sessionNotes = '';
    }

    async function loadSessionChecks() {
        if (!currentSeating) {
            sessionChecks = [];
            return;
        }
        
        loadingChecks = true;
        try {
            const { checksApi } = await import('$lib/api/endpoints/checks.js');
            sessionChecks = await checksApi.getChecksBySeating(currentSeating.id);
        } catch (err) {
            console.error('Failed to load checks:', err);
            sessionChecks = [];
        } finally {
            loadingChecks = false;
        }
    }

    $effect(() => {
        if (currentSeating) {
            customerName = currentSeating.customer_name || '';
            assignedServerId = currentSeating.assigned_server_id;
            sessionNotes = currentSeating.notes || '';
            loadSessionChecks();
        } else {
            resetForm();
            sessionChecks = [];
        }
    });

    let openChecksCount = $derived(sessionChecks.filter(c => c.status !== 'closed').length);
    let canCloseTable = $derived(openChecksCount === 0);
</script>

<div class="h-full bg-white border-l border-gray-200 flex flex-col">
    {#if selectedTable}
        <!-- Header -->
        <div class="p-4 border-b border-gray-200 flex justify-between items-center">
            <h2 class="text-xl font-bold text-gray-900">Table {selectedTable.code}</h2>
            <button 
                class="p-2 hover:bg-gray-100 rounded-lg transition-colors"
                onclick={onClose}
            >
                ✕
            </button>
        </div>

        <!-- Table Info -->
        <div class="p-4 border-b border-gray-200">
            <div class="space-y-2">
                <div class="flex justify-between">
                    <span class="text-gray-600">Capacity:</span>
                    <span class="font-medium">{selectedTable.capacity} seats</span>
                </div>
                {#if selectedTable.section}
                    <div class="flex justify-between">
                        <span class="text-gray-600">Section:</span>
                        <span class="font-medium">{selectedTable.section}</span>
                    </div>
                {/if}
                <div class="flex justify-between">
                    <span class="text-gray-600">Status:</span>
                    <span class="font-medium capitalize 
                        {tableStatus === 'available' ? 'text-green-600' : 
                          tableStatus === 'reserved' ? 'text-yellow-600' : 'text-red-600'}">
                        {tableStatus}
                    </span>
                </div>
                {#if currentSeating?.assigned_server_id}
                    <div class="flex justify-between">
                        <span class="text-gray-600">Server:</span>
                        <span class="font-medium">
                            {staff.find(s => s.id === currentSeating.assigned_server_id)?.name || 'Unknown'}
                        </span>
                    </div>
                {/if}
                {#if currentSeating?.customer_name}
                    <div class="flex justify-between">
                        <span class="text-gray-600">Customer:</span>
                        <span class="font-medium">{currentSeating.customer_name}</span>
                    </div>
                {/if}
            </div>
        </div>

        <!-- Actions -->
        <div class="flex-1 p-4 space-y-4">
            {#if tableStatus === 'available'}
                <h3 class="font-semibold text-gray-900">Start New Session</h3>
                
                <label class="flex items-center space-x-2">
                    <input 
                        type="checkbox" 
                        bind:checked={isReservation}
                        class="rounded border-gray-300"
                    />
                    <span class="text-sm text-gray-700">This is a reservation</span>
                </label>

                {#if isReservation}
                    <input 
                        type="text" 
                        bind:value={customerName}
                        placeholder="Customer name"
                        class="w-full px-3 py-2 border border-gray-300 rounded-lg focus:ring-2 focus:ring-blue-500 focus:border-blue-500"
                    />
                {/if}

                <select 
                    bind:value={assignedServerId}
                    class="w-full px-3 py-2 border border-gray-300 rounded-lg focus:ring-2 focus:ring-blue-500 focus:border-blue-500"
                >
                    <option value={undefined}>Select server (optional)</option>
                    {#each staff.filter(s => s.is_active && (s.role === 'server' || s.role === 'manager')) as staffMember}
                        <option value={staffMember.id}>{staffMember.name} - {staffMember.role}</option>
                    {/each}
                </select>

                <button 
                    class="w-full px-4 py-2 bg-blue-600 text-white rounded-lg hover:bg-blue-700 transition-colors"
                    onclick={startNewSession}
                >
                    {isReservation ? 'Create Reservation' : 'Seat Customers'}
                </button>
            {:else if tableStatus === 'reserved'}
                <h3 class="font-semibold text-gray-900">Reservation Ready</h3>
                
                <div class="p-3 bg-yellow-50 border border-yellow-200 rounded-lg mb-4">
                    <p class="text-sm text-yellow-800">
                        This table has a reservation. Assign a server and seat the customers to start service.
                    </p>
                </div>

                <div class="space-y-3">
                    <input 
                        type="text" 
                        bind:value={customerName}
                        placeholder="Customer name"
                        class="w-full px-3 py-2 border border-gray-300 rounded-lg focus:ring-2 focus:ring-blue-500 focus:border-blue-500"
                    />

                    <select 
                        bind:value={assignedServerId}
                        class="w-full px-3 py-2 border border-gray-300 rounded-lg focus:ring-2 focus:ring-blue-500 focus:border-blue-500"
                    >
                        <option value={undefined}>Select server (required)</option>
                        {#each staff.filter(s => s.is_active && (s.role === 'server' || s.role === 'manager')) as staffMember}
                            <option value={staffMember.id}>{staffMember.name} - {staffMember.role}</option>
                        {/each}
                    </select>

                    <textarea 
                        bind:value={sessionNotes}
                        placeholder="Special requests or notes"
                        rows="2"
                        class="w-full px-3 py-2 border border-gray-300 rounded-lg focus:ring-2 focus:ring-blue-500 focus:border-blue-500"
                    ></textarea>

                    <button 
                        class="w-full px-4 py-2 bg-green-600 text-white rounded-lg hover:bg-green-700 transition-colors disabled:bg-gray-400 disabled:cursor-not-allowed"
                        disabled={!assignedServerId}
                        onclick={seatReservation}
                    >
                        Seat Customers
                    </button>

                    <button 
                        class="w-full px-4 py-2 bg-red-600 text-white rounded-lg hover:bg-red-700 transition-colors"
                        onclick={endSession}
                    >
                        Cancel Reservation
                    </button>
                </div>
            {:else}
                <h3 class="font-semibold text-gray-900">Active Table</h3>
                
                <div class="space-y-3">
                    <input 
                        type="text" 
                        bind:value={customerName}
                        placeholder="Customer name"
                        class="w-full px-3 py-2 border border-gray-300 rounded-lg focus:ring-2 focus:ring-blue-500 focus:border-blue-500"
                    />

                    <select 
                        bind:value={assignedServerId}
                        class="w-full px-3 py-2 border border-gray-300 rounded-lg focus:ring-2 focus:ring-blue-500 focus:border-blue-500"
                    >
                        <option value={undefined}>No server assigned</option>
                        {#each staff.filter(s => s.is_active && (s.role === 'server' || s.role === 'manager')) as staffMember}
                            <option value={staffMember.id}>{staffMember.name} - {staffMember.role}</option>
                        {/each}
                    </select>

                    <textarea 
                        bind:value={sessionNotes}
                        placeholder="Session notes"
                        rows="3"
                        class="w-full px-3 py-2 border border-gray-300 rounded-lg focus:ring-2 focus:ring-blue-500 focus:border-blue-500"
                    ></textarea>

                    <button 
                        class="w-full px-4 py-2 bg-green-600 text-white rounded-lg hover:bg-green-700 transition-colors"
                        onclick={updateSessionDetails}
                    >
                        Update Session
                    </button>

                    {#if !canCloseTable}
                        <div class="p-3 bg-yellow-50 border border-yellow-200 rounded-lg">
                            <p class="text-sm text-yellow-800">
                                Cannot close table: {openChecksCount} open check{openChecksCount !== 1 ? 's' : ''} must be closed first
                            </p>
                        </div>
                    {/if}

                    <button 
                        class="w-full px-4 py-2 bg-red-600 text-white rounded-lg hover:bg-red-700 transition-colors disabled:bg-gray-400 disabled:cursor-not-allowed"
                        disabled={!canCloseTable}
                        onclick={endSession}
                    >
                        Close Table
                    </button>
                </div>
            {/if}
        </div>
    {:else}
        <!-- No table selected -->
        <div class="flex-1 flex items-center justify-center p-4">
            <div class="text-center text-gray-500">
                <p class="text-lg mb-2">Select a table</p>
                <p class="text-sm">Tap on any table to view details</p>
            </div>
        </div>
    {/if}
</div>