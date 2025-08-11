import { useEffect } from 'react';
import { cartController } from '../controllers';
import type { MenuItem } from '../models';

export function useLocalCart() {
    useEffect(() => {
        // Subscribe to cart changes
        const unsubscribe = cartController.subscribe(() => {

        });

        return unsubscribe;
    }, []);

    const addItem = (menuItem: MenuItem, quantity: number = 1) => {
        return cartController.addItem(menuItem, quantity);
    };

    return {
        addItem,
    };
}