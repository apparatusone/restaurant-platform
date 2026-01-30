// JWT validation utilities for frontend
export function isTokenExpired(token: string): boolean {
    try {
        const payload = JSON.parse(atob(token.split('.')[1]));
        const currentTime = Math.floor(Date.now() / 1000);
        return payload.exp < currentTime;
    } catch {
        return true; // Invalid token
    }
}

export function getTokenPayload(token: string): any | null {
    try {
        return JSON.parse(atob(token.split('.')[1]));
    } catch {
        return null;
    }
}

export function clearExpiredAuth() {
    if (typeof window === 'undefined') return;
    
    const token = localStorage.getItem('auth_token');
    if (token && isTokenExpired(token)) {
        localStorage.removeItem('auth_token');
        document.cookie = 'userSession=; path=/; expires=Thu, 01 Jan 1970 00:00:00 GMT';
        localStorage.removeItem('currentUser');
        
        // Redirect to login if not already there
        if (window.location.pathname !== '/login') {
            window.location.href = '/login';
        }
    }
}