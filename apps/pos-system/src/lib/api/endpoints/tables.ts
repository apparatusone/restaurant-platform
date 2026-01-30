import { apiClient } from '../client.js';
import type { Table, TableWithSession, TableSeating } from '$lib/types/index.js';

export const tablesApi = {
    // Get all tables
    async getAllTables(): Promise<Table[]> {
        return apiClient.request<Table[]>('tables/');
    },

    // Get all tables with current session info (for floor view)
    async getTablesWithSessions(): Promise<TableWithSession[]> {
        return apiClient.request<TableWithSession[]>('tables/with-sessions');
    },

    // Get available tables
    async getAvailableTables(): Promise<Table[]> {
        return apiClient.request<Table[]>('tables/available');
    },

    // Get occupied tables
    async getOccupiedTables(): Promise<Table[]> {
        return apiClient.request<Table[]>('tables/occupied');
    },

    // Get table by ID
    async getTable(id: number): Promise<Table> {
        return apiClient.request<Table>(`tables/${id}`);
    },

    // Get table by code
    async getTableByCode(code: string): Promise<Table> {
        return apiClient.request<Table>(`tables/code/${code}`);
    }
};

export const tableSeatingsApi = {
    // Get all active seatings
    async getActiveSeatings(): Promise<TableSeating[]> {
        return apiClient.request<TableSeating[]>('seatings/active');
    },

    // Create new seating
    async createSeating(data: { 
        table_id: number; 
        assigned_server_id?: number;
        notes?: string;
    }): Promise<TableSeating> {
        return apiClient.request<TableSeating>('seatings/', {
            method: 'POST',
            body: JSON.stringify(data)
        });
    },

    // Close seating
    async closeSeating(seatingId: number): Promise<TableSeating> {
        return apiClient.request<TableSeating>(`seatings/${seatingId}/close`, {
            method: 'POST'
        });
    },

    // Update seating
    async updateSeating(seatingId: number, data: Partial<TableSeating>): Promise<TableSeating> {
        return apiClient.request<TableSeating>(`seatings/${seatingId}`, {
            method: 'PUT',
            body: JSON.stringify(data)
        });
    }
};