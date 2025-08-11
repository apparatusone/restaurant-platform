import React, { createContext, useContext } from 'react';
import { useLocalCart } from './hooks/useLocalCart';
import type { LocalCartItem, MenuItem, Order } from './models';

interface CartContextType {
    cartItems: LocalCartItem[];
    addItem: (menuItem: MenuItem, quantity?: number) => LocalCartItem;
    updateQuantity: (menuItemId: number, newQuantity: number) => LocalCartItem | null;
    changeQuantity: (menuItemId: number, delta: number) => LocalCartItem | null;
    removeItem: (menuItemId: number) => boolean;
    clearCart: () => void;
    getItem: (menuItemId: number) => LocalCartItem | undefined;
    prepareForCheckout: () => Order;
    isEmpty: boolean;
    totalItems: number;
    totalPrice: number;
}

const CartContext = createContext<CartContextType | undefined>(undefined);

export const CartProvider: React.FC<{ children: React.ReactNode }> = ({ children }) => {
    const localCart = useLocalCart();

    return (
        <CartContext.Provider value={{
            cartItems: localCart.items,
            addItem: localCart.addItem,
            updateQuantity: localCart.updateQuantity,
            changeQuantity: localCart.changeQuantity,
            removeItem: localCart.removeItem,
            clearCart: localCart.clearCart,
            getItem: localCart.getItem,
            prepareForCheckout: localCart.prepareForCheckout,
            isEmpty: localCart.isEmpty,
            totalItems: localCart.totalItems,
            totalPrice: localCart.totalPrice
        }}>
            {children}
        </CartContext.Provider>
    );
};

export function useCart() {
    const context = useContext(CartContext);
    if (!context) {
        throw new Error('useCart must be used within a CartProvider');
    }
    return context;
}
