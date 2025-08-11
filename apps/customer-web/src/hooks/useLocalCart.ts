// State management layer for cart (model layer)
import { useState, useEffect } from 'react';
import { cartController } from '../controllers';
import type { LocalCartItem, MenuItem, CartSummary } from '../models';

export function useLocalCart() {
    const [items, setItems] = useState<LocalCartItem[]>([]);
    const [summary, setSummary] = useState<CartSummary>(cartController.getSummary());

    useEffect(() => {
        // Subscribe to cart changes
        const unsubscribe = cartController.subscribe(() => {
        setItems(cartController.getItems());
        setSummary(cartController.getSummary());
        });

        return unsubscribe;
    }, []);

    const addItem = (menuItem: MenuItem, quantity: number = 1) => {
        return cartController.addItem(menuItem, quantity);
    };

    const updateQuantity = (menuItemId: number, newQuantity: number) => {
        return cartController.updateQuantity(menuItemId, newQuantity);
    };

    const changeQuantity = (menuItemId: number, delta: number) => {
        return cartController.changeQuantity(menuItemId, delta);
    };

    const removeItem = (menuItemId: number) => {
        return cartController.removeItem(menuItemId);
    };

    const clearCart = () => {
        cartController.clear();
    };

    const getItem = (menuItemId: number) => {
        return cartController.getItem(menuItemId);
    };

    const prepareForCheckout = () => {
        return cartController.prepareForCheckout();
    };

    return {
items,
        summary,
        addItem,
        updateQuantity,
        changeQuantity,
        removeItem,
        clearCart,
        getItem,
        prepareForCheckout,

        isEmpty: summary.isEmpty,
        totalItems: summary.itemCount,
        totalPrice: summary.totalPrice,
    };
}