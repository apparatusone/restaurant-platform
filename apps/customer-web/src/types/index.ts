export interface MenuItem {
  id: number;
  name: string;
  price: number;
  description?: string;
}

export interface CartItem {
  menu_item_id: number;
  name: string;
  amount: number;
  description: string;
  price: number;
}

export interface ApiResponse<T = any> {
  data?: T;
  message?: string;
  detail?: string;
}

export interface CartUpdateResponse {
  quantity: number;
  removed?: boolean;
  message?: string;
}