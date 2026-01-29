import { isTokenExpired, clearExpiredAuth } from '$lib/utils/jwt.js';

export interface User {
    id: string;
    name: string;
    role: 'host' | 'server' | 'manager' | 'kitchen' | 'admin';
}

function getCookieValue(name: string): string | null {
    if (typeof document === 'undefined') return null;
    
    const value = `; ${document.cookie}`;
    const parts = value.split(`; ${name}=`);
    if (parts.length === 2) {
        return parts.pop()?.split(';').shift() || null;
    }
    return null;
}

class AuthState {
    currentUser = $state<User | null>(null);
    token = $state<string | null>(null);
    isAuthenticated = $derived(this.currentUser !== null && this.token !== null);
    private _tokenInterval: ReturnType<typeof setInterval> | null = null;

    constructor() {
        if (typeof window !== 'undefined') {
            clearExpiredAuth();
            // Initialize token state
            const existingToken = localStorage.getItem('auth_token');
            if (existingToken && !isTokenExpired(existingToken)) {
                this.token = existingToken;
                
                // Set token on API client
                import('$lib/api/client.js').then(({ apiClient }) => {
                    apiClient.setAuthToken(existingToken);
                });
                
                // Only hydrate user if token is valid
                const cookieData = getCookieValue('userSession');
                const localData = localStorage.getItem('currentUser');
                try {
                    if (cookieData) {
                        this.currentUser = JSON.parse(decodeURIComponent(cookieData));
                    } else if (localData) {
                        this.currentUser = JSON.parse(localData);
                    }
                } catch {
                    this.logout();
                }
                this.startTokenValidation();
            } else {
                // Ensure clean state if no valid token
                this.logout();
            }

            window.addEventListener('storage', (e) => {
                if (e.key === 'auth_token' && !e.newValue) {
                    this.logout();
                }
            });
        }
    }

    login(user: User) {
        this.currentUser = user;
        if (typeof window !== 'undefined') {
            document.cookie = `userSession=${encodeURIComponent(JSON.stringify(user))}; path=/; max-age=86400; SameSite=Strict`;
            localStorage.setItem('currentUser', JSON.stringify(user));
            const t = localStorage.getItem('auth_token');
            if (t) {
                this.token = t;
                import('$lib/api/client.js').then(({ apiClient }) => {
                    apiClient.setAuthToken(t);
                });
            }
            this.startTokenValidation();
        }
    }

    hydrate(user: User) {
        if (typeof window === 'undefined') {
            this.currentUser = user;
            return;
        }
        const t = localStorage.getItem('auth_token');
        if (t && !isTokenExpired(t)) {
            this.token = t;
            this.currentUser = user;
            import('$lib/api/client.js').then(({ apiClient }) => {
                apiClient.setAuthToken(t);
            });
            this.startTokenValidation();
        } else {
            this.logout();
        }
    }

    private startTokenValidation() {
        if (typeof window === 'undefined') return;
        if (this._tokenInterval) return;
        this._tokenInterval = setInterval(() => {
            const token = localStorage.getItem('auth_token');
            if (!token || isTokenExpired(token)) {
                this.logout();
                if (this._tokenInterval) {
                    clearInterval(this._tokenInterval);
                    this._tokenInterval = null;
                }
            }
        }, 5000);
    }

    logout() {
        this.currentUser = null;
        this.token = null;
        if (typeof window !== 'undefined') {
            document.cookie = 'userSession=; path=/; expires=Thu, 01 Jan 1970 00:00:00 GMT';
            localStorage.removeItem('currentUser');
        }
    }

    switchUser() {
        this.logout();
    }
}

export const authState = new AuthState();