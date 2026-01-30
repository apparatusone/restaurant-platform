<script lang="ts">
    import { checkStore } from '$lib/stores/check.svelte.js';

    interface Props {
        onBack: () => void;
    }

    let { onBack }: Props = $props();
</script>

<div class="flex flex-grow">
    <!-- Left Pane - Check Summary -->
    <section class="w-1/3 bg-gray-50 border-r border-gray-200 flex flex-col p-4">
        <div class="flex items-center justify-between mb-4">
            <h2 class="text-lg font-semibold">Check Summary</h2>
            <button 
                onclick={onBack}
                class="text-blue-500 hover:text-blue-700"
            >
                ← Back
            </button>
        </div>
        
        <div class="space-y-2">
            <div class="text-sm text-gray-600">Check #{checkStore.currentCheck?.id}</div>
            <div class="text-2xl font-bold">${checkStore.checkTotal.toFixed(2)}</div>
        </div>
    </section>

    <!-- Middle Pane - Payment Options -->
    <section class="w-1/3 bg-white border-r border-gray-200 flex flex-col p-4">
        <h2 class="text-lg font-semibold mb-4">Payment Options</h2>
        
        <div class="space-y-4">
            <button
                onclick={async () => {
                    if (!checkStore.currentCheck) return;
                    try {
                        const { checksApi } = await import('$lib/api/endpoints/checks.js');
                        const { paymentsApi } = await import('$lib/api/endpoints/payments.js');
                        
                        // Reload check to get latest status from database
                        const latestCheck = await checksApi.getCheck(checkStore.currentCheck.id);
                        
                        console.log('Processing payment for check:', latestCheck.id);
                        console.log('Amount:', checkStore.checkTotal);
                        console.log('Check status (from DB):', latestCheck.status);
                        
                        // Check if already paid
                        if (latestCheck.status === 'paid' || latestCheck.status === 'closed') {
                            alert('This check has already been paid!');
                            // Reload all checks to update UI
                            if (latestCheck.seating_id) {
                                await checkStore.loadChecksForSeating(latestCheck.seating_id);
                            }
                            onBack();
                            return;
                        }
                        
                        // Check for existing payments and handle orphaned state
                        try {
                            const summary = await paymentsApi.getCheckPaymentSummary(latestCheck.id);
                            const isFullyPaid = Number(summary?.total_paid ?? 0) >= Number(latestCheck.total_amount ?? 0);
                            if (isFullyPaid) {
                                // Check is fully paid but status wasn't updated - close it
                                try {
                                    const closed = await checksApi.closeCheck(latestCheck.id);
                                    alert('Check was already paid. Status has been updated.');
                                    if (closed.seating_id) {
                                        await checkStore.loadChecksForSeating(closed.seating_id);
                                    }
                                    onBack();
                                    return;
                                } catch {
                                    // Ignore close errors and continue with normal flow
                                }
                            }
                        } catch (err) {
                            console.warn('Could not check payment summary:', err);
                        }
                        
                        // Recalculate check totals from order items
                        console.log('Recalculating check totals...');
                        const updatedCheck = await checksApi.recalculateCheck(checkStore.currentCheck.id);
                        console.log('Check after recalculation:', updatedCheck);
                        console.log('Total amount:', updatedCheck.total_amount);
                        
                        // Verify the check has a valid total
                        if (!updatedCheck.total_amount || updatedCheck.total_amount === 0) {
                            throw new Error('Check total is $0 - cannot process payment');
                        }
                        
                        // Update store with recalculated check
                        checkStore.checks = checkStore.checks.map(c => 
                            c.id === updatedCheck.id ? updatedCheck : c
                        );
                        
                        // Update check to READY status if not already
                        if (updatedCheck.status !== 'ready') {
                            const readyCheck = await checksApi.updateCheck(checkStore.currentCheck.id, { status: 'ready' });
                            // Update store again with ready status
                            checkStore.checks = checkStore.checks.map(c => 
                                c.id === readyCheck.id ? readyCheck : c
                            );
                        }
                        
                        // Quick pay with cash for testing - use the updated check's total
                        console.log('Processing payment for amount:', updatedCheck.total_amount);
                        await checkStore.processPayment(
                            Number(updatedCheck.total_amount),
                            'cash'
                        );
                        
                        // Reload checks to get updated status
                        if (updatedCheck.seating_id) {
                            await checkStore.loadChecksForSeating(updatedCheck.seating_id);
                        }
                        
                        alert('Payment processed successfully!');
                        onBack();
                    } catch (error) {
                        console.error('Payment failed:', error);
                        const errorMsg = error instanceof Error ? error.message : JSON.stringify(error);
                        alert('Payment failed: ' + errorMsg);
                    }
                }}
                disabled={!checkStore.currentCheck}
                class="w-full px-6 py-4 bg-green-600 text-white rounded-lg hover:bg-green-700 disabled:bg-gray-300 disabled:cursor-not-allowed text-lg font-semibold"
            >
                Quick Pay (Cash)
            </button>
            
            <div class="text-sm text-gray-500 text-center">
                For testing: Automatically pays the full amount with cash
            </div>
        </div>
    </section>

    <!-- Right Pane - Receipt/Confirmation -->
    <section class="w-1/3 bg-gray-50 flex flex-col p-4">
        <h2 class="text-lg font-semibold mb-4">Receipt</h2>
        <!-- Receipt details will go here -->
    </section>
</div>
