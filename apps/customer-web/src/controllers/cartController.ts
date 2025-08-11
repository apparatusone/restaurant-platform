// Cart state management and persistence
import type { LocalCartItem, MenuItem } from '../models';
import { CartService } from '../services';

export class CartController {
    private items: Map<number, LocalCartItem> = new Map();
    private listeners: Set<() => void> = new Set();

    // Observer pattern for state changes
    subscribe(listener: () => void): () => void {
        this.listeners.add(listener);
        return () => this.listeners.delete(listener);
    }

    private notify(): void {
        this.listeners.forEach(listener => listener());
    }

    // State modification methods (using service layer)
    addItem(menuItem: MenuItem, quantity: number = 1): LocalCartItem {
        const item = CartService.addItemToCart(this.items, menuItem, quantity);
        this.notify();
        return item;
    }
}

// Singleton instance
export const cartController = new CartController();