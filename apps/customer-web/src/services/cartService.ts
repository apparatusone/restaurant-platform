// Cart business logic
import type { MenuItem, LocalCartItem, Order, OrderItem, CartSummary } from '../models';

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

    static updateItemQuantity(
        items: Map<number, LocalCartItem>,
        menuItemId: number,
        newQuantity: number
    ): LocalCartItem | null {
        const item = items.get(menuItemId);
        if (!item) return null;

        if (newQuantity <= 0) {
            items.delete(menuItemId);
            return null;
        }

        item.quantity = newQuantity;
        return item;
    }

    static changeItemQuantity(
        items: Map<number, LocalCartItem>,
        menuItemId: number,
        delta: number
    ): LocalCartItem | null {
        const item = items.get(menuItemId);
        if (!item) return null;

        return this.updateItemQuantity(items, menuItemId, item.quantity + delta);
    }

    static removeItem(items: Map<number, LocalCartItem>, menuItemId: number): boolean {
        return items.delete(menuItemId);
    }

    static calculateSummary(items: LocalCartItem[]): CartSummary {
        const totalItems = items.reduce((sum, item) => sum + item.quantity, 0);
        const totalPrice = items.reduce((sum, item) => sum + (item.price * item.quantity), 0);
        
        return {
            itemCount: totalItems,
            totalPrice,
            isEmpty: items.length === 0,
        };
    }

    static prepareOrder(cartItems: LocalCartItem[]): Order {
        const items: OrderItem[] = cartItems.map(item => ({
        menu_item_id: item.id,
        quantity: item.quantity,
        price: item.price,
        }));

        const total_price = cartItems.reduce((sum, item) => sum + (item.price * item.quantity), 0);
        const item_count = cartItems.reduce((sum, item) => sum + item.quantity, 0);

        return {
            items,
            total_price,
            item_count,
        };
    }
}