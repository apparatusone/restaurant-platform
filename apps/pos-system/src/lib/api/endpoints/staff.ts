import { apiClient } from '../client.js';
import type { Staff } from '$lib/types/index.js';

export const staffApi = {
    async getAllStaff(): Promise<Staff[]> {
        return apiClient.request<Staff[]>('staff/');
    },

    async getStaff(id: number): Promise<Staff> {
        return apiClient.request<Staff>(`staff/${id}`);
    },

    async createStaff(data: any): Promise<Staff> {
        return apiClient.request<Staff>('staff/', {
            method: 'POST',
            body: JSON.stringify(data)
        });
    },

    async updateStaff(id: number, data: any): Promise<Staff> {
        return apiClient.request<Staff>(`staff/${id}`, {
            method: 'PUT',
            body: JSON.stringify(data)
        });
    },

    async deleteStaff(id: number): Promise<void> {
        return apiClient.request<void>(`staff/${id}`, {
            method: 'DELETE'
        });
    }
};