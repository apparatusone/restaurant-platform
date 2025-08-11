// Cart state management and persistence
import type { LocalCartItem, MenuItem, Order, CartSummary } from '../models';
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

    // methods to get data
    getItems(): LocalCartItem[] {
        return Array.from(this.items.values());
    }

    getItem(menuItemId: number): LocalCartItem | undefined {
        return this.items.get(menuItemId);
    }

    getSummary(): CartSummary {
        return CartService.calculateSummary(this.getItems());
    }

    // methods to modify state
    addItem(menuItem: MenuItem, quantity: number = 1): LocalCartItem {
        const item = CartService.addItemToCart(this.items, menuItem, quantity);
        this.notify();
        return item;
    }

    updateQuantity(menuItemId: number, newQuantity: number): LocalCartItem | null {
        const result = CartService.updateItemQuantity(this.items, menuItemId, newQuantity);
        this.notify();
        return result;
    }

    changeQuantity(menuItemId: number, delta: number): LocalCartItem | null {
        const result = CartService.changeItemQuantity(this.items, menuItemId, delta);
        this.notify();
        return result;
    }

    removeItem(menuItemId: number): boolean {
        const result = CartService.removeItem(this.items, menuItemId);
        if (result) this.notify();
        
        return result;
    }

    clear(): void {
        this.items.clear();
        this.notify();
    }

    prepareForCheckout(): Order {
        return CartService.prepareOrder(this.getItems());
    }

    // store/pull cart in local storage
    serialize(): string {
        return JSON.stringify(Array.from(this.items.entries()));
    }

    deserialize(data: string): void {
        try {
        const entries = JSON.parse(data) as [number, LocalCartItem][];
            this.items = new Map(entries);
            this.notify();
        } catch (error) {
            console.error('Failed to deserialize cart:', error);
        }
    }

    saveToStorage(): void {
        try {
            localStorage.setItem('cart', this.serialize());
        } catch (error) {
            console.error('Failed to save cart to storage:', error);
        }
    }

    loadFromStorage(): void {
        try {
            const data = localStorage.getItem('cart');
        if (data) {
            this.deserialize(data);
        }
        } catch (error) {
            console.error('Failed to load cart from storage:', error);
        }
    }
}

// Singleton instance
export const cartController = new CartController();