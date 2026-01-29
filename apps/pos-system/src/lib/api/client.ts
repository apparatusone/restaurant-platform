import { config } from '../config.js';

// Main API client class
export class POSApiClient {
    private authToken: string | null = null;

    constructor() {
        // Initialize token from localStorage if available
        if (typeof window !== 'undefined') {
            const existingToken = localStorage.getItem('auth_token');
            if (existingToken) {
                this.authToken = existingToken;
            }
        }
    }

    setAuthToken(token: string) {
        this.authToken = token;
    }

    async request<T>(endpoint: string, options: RequestInit = {}): Promise<T> {
        const url = `${config.serverUrl}/${endpoint}`;
        
        const headers = new Headers(options.headers as HeadersInit);
        if (!headers.has('Content-Type')) {
            headers.set('Content-Type', 'application/json');
        }

        if (this.authToken) {
            headers.set('Authorization', `Bearer ${this.authToken}`);
        }

        const response = await fetch(url, {
            ...options,
            headers,
        });

        if (!response.ok) {
            // Try to get error details from response body
            let errorDetail = response.statusText;
            try {
                const errorBody = await response.json();
                if (typeof errorBody === 'string') {
                    errorDetail = errorBody;
                } else if (errorBody.detail) {
                    errorDetail = typeof errorBody.detail === 'string' 
                        ? errorBody.detail 
                        : JSON.stringify(errorBody.detail);
                } else if (errorBody.message) {
                    errorDetail = errorBody.message;
                } else {
                    errorDetail = JSON.stringify(errorBody);
                }
            } catch {
                // If response body is not JSON, use statusText
            }
            throw new Error(`API Error: ${response.status} - ${errorDetail}`);
        }

        // Handle empty responses (common for DELETE operations)
        const text = await response.text();
        if (!text) {
            return undefined as T;
        }
        
        return JSON.parse(text);
    }
}

// Export singleton instance
export const apiClient = new POSApiClient();