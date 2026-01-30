import { apiClient } from '../client.js';

export interface Ingredient {
    id: number;
    name: string;
    quantity_on_hand: number;
    apriltag_id: number | null;
    restaurant_id: number;
    unit: string;
    reorder_point: number;
}

export interface IngredientUpdate {
    name?: string;
    quantity_on_hand?: number;
    apriltag_id?: number | null;
}

export const ingredientsApi = {
    async getAll(): Promise<Ingredient[]> {
        return apiClient.request<Ingredient[]>('ingredients');
    },

    async update(id: number, data: IngredientUpdate): Promise<Ingredient> {
        return apiClient.request<Ingredient>(`ingredients/${id}`, {
            method: 'PUT',
            body: JSON.stringify(data)
        });
    }
};
