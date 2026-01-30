import { apiClient } from '../client.js';
import type { CheckItem } from '../../types/restaurant.js';

class CheckItemsAPI {
    // Get check items for a specific check
    async getCheckItemsForCheck(checkId: number): Promise<CheckItem[]> {
        return apiClient.request<CheckItem[]>(`check-items?check_id=${checkId}`);
    }

    // Get all check items, optionally filtered by status
    async getAllCheckItems(status?: string): Promise<CheckItem[]> {
        const params = status ? `?status=${status}` : '';
        return apiClient.request<CheckItem[]>(`check-items${params}`);
    }

    // Get check item by ID
    async getCheckItem(id: number): Promise<CheckItem> {
        return apiClient.request<CheckItem>(`check-items/${id}`);
    }

    // Create new check item directly for a check
    async createCheckItemForCheck(data: {
        check_id: number;
        menu_item_id: number;
        quantity: number;
        special_instructions?: string;
    }): Promise<CheckItem> {
        return apiClient.request<CheckItem>(`checks/${data.check_id}/items`, {
            method: 'POST',
            body: JSON.stringify({
                menu_item_id: data.menu_item_id,
                quantity: data.quantity,
                special_instructions: data.special_instructions
            })
        });
    }

    // Update check item (including status changes)
    async updateCheckItem(id: number, data: { 
        status?: 'pending' | 'preparing' | 'ready' | 'served'; 
        quantity?: number; 
        special_instructions?: string 
    }): Promise<CheckItem> {
        return apiClient.request<CheckItem>(`check-items/${id}`, {
            method: 'PUT',
            body: JSON.stringify(data)
        });
    }

    // Mark item as ready
    async markItemReady(id: number): Promise<CheckItem> {
        return apiClient.request<CheckItem>(`check-items/${id}/ready`, {
            method: 'PUT'
        });
    }

    // Delete check item (only pending items)
    async deleteCheckItem(id: number): Promise<void> {
        return apiClient.request<void>(`check-items/${id}`, {
            method: 'DELETE'
        });
    }
}

export const checkItemsApi = new CheckItemsAPI();
