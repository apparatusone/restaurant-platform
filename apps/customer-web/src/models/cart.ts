// Cart data models and interfaces
export interface LocalCartItem {
    id: number;
    name: string;
    price: number;
    description: string;
    quantity: number;
}

export interface OrderItem {
    menu_item_id: number;
    quantity: number;
    price: number;
}

export interface Order {
    items: OrderItem[];
    total_price: number;
    item_count: number;
}

export interface CartSummary {
    itemCount: number;
    totalPrice: number;
    isEmpty: boolean;
}