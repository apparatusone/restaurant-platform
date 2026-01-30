import { apiClient } from '../client.js';
import type { Check, CheckItem } from '../../types/restaurant.js';

export const checksApi = {
    // Get all checks
    async getAllChecks(): Promise<Check[]> {
        return apiClient.request<Check[]>('checks/');
    },

    // Get open checks
    async getOpenChecks(): Promise<Check[]> {
        return apiClient.request<Check[]>('checks/open');
    },

    // Get checks by seating
    async getChecksBySeating(seatingId: number): Promise<Check[]> {
        return apiClient.request<Check[]>(`checks/seating/${seatingId}`);
    },

    // Get check by ID
    async getCheck(id: number): Promise<Check> {
        return apiClient.request<Check>(`checks/${id}`);
    },

    // Create new check
    async createCheck(data: {
        seating_id?: number;
        is_virtual?: boolean;
        subtotal?: number;
        tax_amount?: number;
        tip_amount?: number;
        total_amount?: number;
    }): Promise<Check> {
        return apiClient.request<Check>('checks/', {
            method: 'POST',
            body: JSON.stringify(data)
        });
    },

    // Update check
    async updateCheck(id: number, data: Partial<Check>): Promise<Check> {
        return apiClient.request<Check>(`checks/${id}`, {
            method: 'PUT',
            body: JSON.stringify(data)
        });
    },

    // Submit check to kitchen
    async submitCheck(id: number): Promise<Check> {
        return apiClient.request<Check>(`checks/${id}/submit`, {
            method: 'POST'
        });
    },

    // Idempotent send-to-kitchen (submit if needed, promote pending items)
    async sendToKitchen(id: number): Promise<Check> {
        return apiClient.request<Check>(`checks/${id}/send`, {
            method: 'POST'
        });
    },

    // Add item to check
    async addItemToCheck(checkId: number, menuItemId: number, quantity: number = 1): Promise<CheckItem> {
        return apiClient.request<CheckItem>(`checks/${checkId}/items`, {
            method: 'POST',
            body: JSON.stringify({
                menu_item_id: menuItemId,
                quantity
            })
        });
    },

    // Recalculate check totals
    async recalculateCheck(checkId: number): Promise<Check> {
        return apiClient.request<Check>(`checks/${checkId}/recalculate`, {
            method: 'POST'
        });
    },

    // Close check
    async closeCheck(checkId: number): Promise<Check> {
        return apiClient.request<Check>(`checks/${checkId}/close`, {
            method: 'PUT'
        });
    },

    // Delete check
    async deleteCheck(checkId: number): Promise<void> {
        return apiClient.request<void>(`checks/${checkId}`, {
            method: 'DELETE'
        });
    },
};
