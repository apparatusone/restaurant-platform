// Cart business logic
import type { MenuItem, LocalCartItem } from '../models';

export class CartService {
    // Business logic for cart operations
    
    static addItemToCart(
        existingItems: Map<number, LocalCartItem>,
        menuItem: MenuItem,
        quantity: number = 1
    ): LocalCartItem {
        const existingItem = existingItems.get(menuItem.id);
        
        if (existingItem) {
            existingItem.quantity += quantity;
            return existingItem;
        } else {
            const newItem: LocalCartItem = {
                id: menuItem.id,
                name: menuItem.name,
                price: menuItem.price,
                description: menuItem.description || '',
                quantity: quantity,
            };
            existingItems.set(menuItem.id, newItem);
            return newItem;
        }
    }
}