export interface MenuItem {
    id: number;
    name: string;
    price: number;
    description?: string;
    calories: number;
    food_category: 'vegetarian' | 'vegan' | 'gluten_free' | 'regular';
}

export interface MenuCategory {
    id: string;
    name: string;
    items: MenuItem[];
}

export interface Table {
    id: number;
    code: string;
    capacity: number;
    section?: string;
    is_outdoor: boolean;
    notes?: string;
}

export interface TableWithSession extends Table {
    current_seating_id?: number;
}

export interface TableSeating {
    id: number;
    table_id: number;
    customer_name?: string;
    assigned_server_id?: number;
    is_reservation?: boolean;
    notes?: string;
    opened_at: string;
    closed_at?: string;
}

export type TableStatus = 'available' | 'occupied' | 'reserved';

export interface Staff {
    id: number;
    staff_id: string;
    name: string;
    role: 'host' | 'server' | 'manager' | 'kitchen' | 'admin';
    is_active: boolean;
    failed_attempts: number;
    last_login?: string;
}

export interface Check {
    id: number;
    seating_id?: number;
    is_virtual: boolean;
    status: 'open' | 'sent' | 'ready' | 'paid' | 'closed';
    subtotal: number;
    tax_amount: number;
    tip_amount: number;
    total_amount: number;
    created_at: string;
    updated_at: string;
    submitted_at?: string;
    paid_at?: string;
}

export interface CheckItem {
    id: number;
    check_id: number;
    menu_item_id: number;
    quantity: number;
    unit_price: number;
    total_price: number;
    special_instructions?: string;
    status: 'pending' | 'preparing' | 'ready' | 'served';
    created_at: string;
    updated_at: string;
    menu_item?: MenuItem;
}

export interface Ticket {
    check: Check;
    items: CheckItem[];
}

export interface Payment {
    id: number;
    check_id: number;
    amount: number;
    payment_type: 'cash' | 'credit_card' | 'debit_card';
    status: 'pending' | 'completed' | 'failed' | 'refunded';
    card_number?: string | null;
    payment_date: string;
}