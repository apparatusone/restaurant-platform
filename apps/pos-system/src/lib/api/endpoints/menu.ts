import type { MenuItem } from '$lib/types/index.js';
import { apiClient } from '../client.js';

export class MenuAPI {
    constructor(private client: typeof apiClient) {}

    async getMenuItems(): Promise<MenuItem[]> {
        return this.client.request<MenuItem[]>('menu-items/');
    }

    async getMenuItemsByCategory(category: string): Promise<MenuItem[]> {
        return this.client.request<MenuItem[]>(`menu-items?filter_category=${category}`);
    }

    async getMenuItem(id: number): Promise<MenuItem> {
        return this.client.request<MenuItem>(`menu-items/${id}`);
    }
}

// Export singleton instance
export const menuAPI = new MenuAPI(apiClient);