import React, { createContext, useContext } from 'react';
import { useLocalCart } from './hooks/useLocalCart';
import type { LocalCartItem, MenuItem, Order } from './models';

interface CartContextType {
    cartItems: LocalCartItem[];
    addItem: (menuItem: MenuItem, quantity?: number) => LocalCartItem;
}

const CartContext = createContext<CartContextType | undefined>(undefined);

export const CartProvider: React.FC<{ children: React.ReactNode }> = ({ children }) => {
    const localCart = useLocalCart();

    return (
        <CartContext.Provider value={{
            cartItems: localCart.items,
            addItem: localCart.addItem,
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
