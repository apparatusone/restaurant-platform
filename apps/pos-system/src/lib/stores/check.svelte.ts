import type { Check, CheckItem } from '$lib/types/index.js';

export class CheckStore {
    // Direct state - no private properties or getters
    checks = $state<Check[]>([]);
    currentCheckId = $state<number | null>(null);
    checkItemsByCheck = $state<Map<number, CheckItem[]>>(new Map());
    loading = $state<boolean>(false);
    error = $state<string | null>(null);

    // Derived states - auto-computed
    currentCheck = $derived(this.checks.find(c => c.id === this.currentCheckId) ?? null);
    checkItems = $derived(this.currentCheckId ? (this.checkItemsByCheck.get(this.currentCheckId) ?? []) : []);
    pendingItems = $derived(this.checkItems.filter(item => item.status === 'pending'));
    preparingItems = $derived(this.checkItems.filter(item => item.status === 'preparing'));
    readyItems = $derived(this.checkItems.filter(item => item.status === 'ready'));
    canSendToKitchen = $derived(this.pendingItems.length > 0);
    canProcessPayment = $derived(this.currentCheck?.status === 'ready');
    hasMultipleChecks = $derived(this.checks.length > 1);
    canCloseCheck = $derived(
        this.currentCheck && this.checkItems.length > 0 && 
        this.checkItems.every(item => item.status === 'ready')
    );

    // Actions
    
    /**
     * Load checks for a seating - always fetches fresh data
     */
    async loadChecksForSeating(seatingId: number) {
        try {
            this.loading = true;
            this.error = null;
            
            const { checksApi } = await import('$lib/api/endpoints/checks.js');
            const fetchedChecks = await checksApi.getChecksBySeating(seatingId);
            
            this.checks = fetchedChecks;
            
            // Load check items for all checks
            await this.loadAllCheckItems();
        } catch (err) {
            this.error = err instanceof Error ? err.message : 'Failed to load checks';
            console.error('Failed to load checks:', err);
        } finally {
            this.loading = false;
        }
    }
    
    /**
     * Load check items for all checks
     */
    private async loadAllCheckItems() {
        try {
            const { checkItemsApi } = await import('$lib/api/endpoints/check-items.js');
            
            // Create new Map to trigger reactivity
            const newMap = new Map(this.checkItemsByCheck);
            
            for (const check of this.checks) {
                const items = await checkItemsApi.getCheckItemsForCheck(check.id);
                newMap.set(check.id, items || []);
            }
            
            // Reassign to trigger reactivity
            this.checkItemsByCheck = newMap;
        } catch (error) {
            console.error('Failed to load check items:', error);
        }
    }
    
    /**
     * Select a check by ID
     */
    async selectCheck(checkId: number | null) {
        if (checkId === null) {
            this.currentCheckId = null;
            return;
        }
        
        // Check if check exists in current checks
        const check = this.checks.find(c => c.id === checkId);
        if (!check) {
            console.warn(`Check ${checkId} not found in current checks`);
            return;
        }
        
        this.currentCheckId = checkId;
        
        // Load check items if not already loaded
        if (!this.checkItemsByCheck.has(checkId)) {
            await this.loadCheckItemsForCheck(checkId);
        }
    }
    
    /**
     * Load check items for a specific check
     */
    private async loadCheckItemsForCheck(checkId: number) {
        try {
            const { checkItemsApi } = await import('$lib/api/endpoints/check-items.js');
            const items = await checkItemsApi.getCheckItemsForCheck(checkId);
            
            // Create new Map to trigger reactivity
            const newMap = new Map(this.checkItemsByCheck);
            newMap.set(checkId, items || []);
            this.checkItemsByCheck = newMap;
        } catch (error) {
            console.error('Failed to load check items:', error);
        }
    }
    
    /**
     * Add a new check
     */
    addCheck(check: Check) {
        this.checks = [...this.checks, check];
        
        // Create new Map to trigger reactivity
        const newMap = new Map(this.checkItemsByCheck);
        newMap.set(check.id, []);
        this.checkItemsByCheck = newMap;
        
        this.currentCheckId = check.id;
    }

    async addCheckItem(item: { check_id: number; menu_item_id: number; quantity: number; special_instructions?: string }) {
        // Add item via API
        const { checkItemsApi } = await import('$lib/api/endpoints/check-items.js');
        const newItem = await checkItemsApi.createCheckItemForCheck(item);
        
        // Update local state with new Map to trigger reactivity
        const currentItems = this.checkItemsByCheck.get(item.check_id) ?? [];
        const newMap = new Map(this.checkItemsByCheck);
        newMap.set(item.check_id, [...currentItems, newItem]);
        this.checkItemsByCheck = newMap;
    }

    async deleteMenuItem(checkItemId: number): Promise<boolean> {
        if (!this.currentCheckId) return false;
        
        const item = this.checkItems.find(i => i.id === checkItemId);
        if (!item || !this.canDeleteMenuItem(checkItemId)) return false;
        
        const { checkItemsApi } = await import('$lib/api/endpoints/check-items.js');
        const currentItems = this.checkItemsByCheck.get(this.currentCheckId) ?? [];
        
        if (item.quantity > 1) {
            // Decrement quantity via API
            await checkItemsApi.updateCheckItem(checkItemId, { 
                quantity: item.quantity - 1 
            });
            // Update local state - reassign Map for reactivity
            const newMap = new Map(this.checkItemsByCheck);
            newMap.set(
                this.currentCheckId,
                currentItems.map(i => i.id === checkItemId ? { ...i, quantity: i.quantity - 1 } : i)
            );
            this.checkItemsByCheck = newMap;
        } else {
            // Delete entire item via API
            await checkItemsApi.deleteCheckItem(checkItemId);
            // Remove from local state - reassign Map for reactivity
            const newMap = new Map(this.checkItemsByCheck);
            newMap.set(
                this.currentCheckId,
                currentItems.filter(i => i.id !== checkItemId)
            );
            this.checkItemsByCheck = newMap;
        }
        return true;
    }

    async deleteCheck(checkId: number): Promise<boolean> {
        if (!this.canDelete(checkId)) return false;
        
        const { checksApi } = await import('$lib/api/endpoints/checks.js');
        await checksApi.deleteCheck(checkId);
        
        // Update local state
        this.checks = this.checks.filter(c => c.id !== checkId);
        
        // Reassign Map for reactivity
        const newMap = new Map(this.checkItemsByCheck);
        newMap.delete(checkId);
        this.checkItemsByCheck = newMap;
        
        // Handle current check cleanup
        if (this.currentCheckId === checkId) {
            this.currentCheckId = this.checks[0]?.id ?? null;
        }
        return true;
    }

    /**
     * Returns true if the check can be deleted (no check items, status is open)
     */
    canDelete(checkId: number): boolean {
        const check = this.checks.find((c: Check) => c.id === checkId);
        if (!check) return false;
        if (check.status !== 'open') return false;
        
        // Check if there are any check items
        const items = this.checkItemsByCheck.get(checkId) ?? [];
        return items.length === 0;
    }

    /**
     * Returns true if the menu item (check item) can be deleted (status is pending)
     */
    canDeleteMenuItem(checkItemId: number): boolean {
        const item = this.checkItems.find(i => i.id === checkItemId);
        if (!item) return false;
        // Can only delete items that haven't been sent to kitchen
        return item.status === 'pending';
    }

    clear() {
        this.checks = [];
        this.currentCheckId = null;
        this.checkItemsByCheck = new Map();
        this.loading = false;
        this.error = null;
    }

    async sendAllPendingItems(): Promise<boolean> {
        if (!this.canSendToKitchen || !this.currentCheckId) return false;
        
        try {
            const { checkItemsApi } = await import('$lib/api/endpoints/check-items.js');
            
            // Send all pending items
            await Promise.all(
                this.pendingItems.map(item => 
                    checkItemsApi.updateCheckItem(item.id, { status: 'preparing' })
                )
            );
            
            // Update local state - reassign Map for reactivity
            const currentItems = this.checkItemsByCheck.get(this.currentCheckId) ?? [];
            const newMap = new Map(this.checkItemsByCheck);
            newMap.set(
                this.currentCheckId,
                currentItems.map(item => item.status === 'pending' ? { ...item, status: 'preparing' } : item)
            );
            this.checkItemsByCheck = newMap;
            
            return true;
        } catch (error) {
            console.error('Failed to send items:', error);
            throw error;
        }
    }

    async sendToKitchen(): Promise<boolean> {
        if (!this.canSendToKitchen || !this.currentCheckId) return false;
        
        try {
            this.loading = true;
            this.error = null;
            
            const { checksApi } = await import('$lib/api/endpoints/checks.js');

            // Server owns the workflow: submit if needed + promote pending items.
            const updatedCheck = await checksApi.sendToKitchen(this.currentCheckId);
            this.checks = this.checks.map(c => c.id === updatedCheck.id ? updatedCheck : c);

            // Refresh items from backend (source of truth)
            const { checkItemsApi } = await import('$lib/api/endpoints/check-items.js');
            const items = await checkItemsApi.getCheckItemsForCheck(this.currentCheckId);
            const newMap = new Map(this.checkItemsByCheck);
            newMap.set(this.currentCheckId, items || []);
            this.checkItemsByCheck = newMap;
            
            return true;
        } catch (error) {
            this.error = error instanceof Error ? error.message : 'Failed to send to kitchen';
            console.error('Failed to send to kitchen:', error);
            throw error;
        } finally {
            this.loading = false;
        }
    }

    async processPayment(amount: number, paymentType: 'cash' | 'credit_card' | 'debit_card', cardNumber?: string): Promise<boolean> {
        if (!this.currentCheck) return false;
        
        try {
            const { paymentsApi } = await import('$lib/api/endpoints/payments.js');
            await paymentsApi.processPayment({
                check_id: this.currentCheck.id,
                amount: amount,
                payment_type: paymentType,
                card_number: cardNumber
            });
            
            const { checksApi } = await import('$lib/api/endpoints/checks.js');

            let closedCheck;
            try {
                closedCheck = await checksApi.closeCheck(this.currentCheck.id);
            } catch (err) {
                const msg = err instanceof Error ? err.message : String(err);
                if (msg.includes('must be paid before closing')) {
                    await checksApi.updateCheck(this.currentCheck.id, { status: 'paid' });
                    closedCheck = await checksApi.closeCheck(this.currentCheck.id);
                } else {
                    throw err;
                }
            }
            
            // Update local state - ensure reactivity by creating new array
            this.checks = this.checks.map(c => c.id === closedCheck.id ? closedCheck : c);
            
            console.log('Check closed:', closedCheck.id, 'Status:', closedCheck.status);
            console.log('All checks:', this.checks.map(c => ({ id: c.id, status: c.status })));
            
            return true;
        } catch (error) {
            console.error('Failed to process payment:', error);
            throw error;
        }
    }

    checkTotal = $derived(
        this.checkItems.reduce((total, item) => {
            return total + (Number(item.total_price || item.unit_price * item.quantity));
        }, 0)
    );
}

export const checkStore = new CheckStore();