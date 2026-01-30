<script lang="ts">
    import { checkStore } from '$lib/stores/check.svelte.js';

    interface Props {
        isOpen: boolean;
        onClose: () => void;
    }

    let { isOpen = false, onClose }: Props = $props();
    
    let paymentMethod = $state<'cash' | 'credit_card' | 'debit_card'>('cash');
    let paymentAmount = $state<number>(0);
    let notes = $state<string>('');
    let processing = $state<boolean>(false);

    // Set default payment amount to check total
    $effect(() => {
        if (isOpen) {
            paymentAmount = checkStore.checkTotal;
        }
    });

    async function processPayment() {
        if (!paymentAmount || paymentAmount <= 0) return;
        
        processing = true;
        try {
            await checkStore.processPayment(paymentAmount, paymentMethod);
            onClose();
        } catch (error) {
            console.error('Payment failed:', error);
        } finally {
            processing = false;
        }
    }
</script>

{#if isOpen}
    <div class="fixed inset-0 bg-black bg-opacity-50 flex items-center justify-center z-50">
        <div class="bg-white rounded-lg p-6 w-96 max-w-full mx-4">
            <h2 class="text-xl font-bold mb-4">Process Payment</h2>
            
            <div class="space-y-4">
                <div>
                    <label for="total-amount" class="block text-sm font-medium mb-1">Total Amount</label>
                    <div id="total-amount" class="text-2xl font-bold text-green-600">
                        ${checkStore.checkTotal.toFixed(2)}
                    </div>
                </div>

                <div>
                    <label for="payment-amount" class="block text-sm font-medium mb-1">Payment Amount</label>
                    <input 
                        id="payment-amount"
                        type="number" 
                        bind:value={paymentAmount}
                        step="0.01"
                        min="0"
                        class="w-full border rounded px-3 py-2"
                        placeholder="Enter payment amount"
                    />
                </div>

                <div>
                    <label for="payment-method" class="block text-sm font-medium mb-1">Payment Method</label>
                    <select 
                        id="payment-method"
                        bind:value={paymentMethod} 
                        class="w-full border rounded px-3 py-2"
                    >
                        <option value="cash">Cash</option>
                        <option value="credit_card">Credit Card</option>
                        <option value="debit_card">Debit Card</option>
                    </select>
                </div>

                <div>
                    <label for="payment-notes" class="block text-sm font-medium mb-1">Notes (Optional)</label>
                    <textarea 
                        id="payment-notes"
                        bind:value={notes}
                        class="w-full border rounded px-3 py-2 h-20"
                        placeholder="Payment notes..."
                    ></textarea>
                </div>
            </div>

            <div class="flex gap-2 mt-6">
                <button 
                    onclick={onClose}
                    class="flex-1 px-4 py-2 border rounded hover:bg-gray-50"
                    disabled={processing}
                >
                    Cancel
                </button>
                <button 
                    onclick={processPayment}
                    class="flex-1 px-4 py-2 bg-green-500 text-white rounded hover:bg-green-600 disabled:opacity-50"
                    disabled={processing || !paymentAmount || paymentAmount <= 0}
                >
                    {processing ? 'Processing...' : `Pay $${paymentAmount.toFixed(2)}`}
                </button>
            </div>
        </div>
    </div>
{/if}
