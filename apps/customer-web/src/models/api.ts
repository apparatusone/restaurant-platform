// API response models
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