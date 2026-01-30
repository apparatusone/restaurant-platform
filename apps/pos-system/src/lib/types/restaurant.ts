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

