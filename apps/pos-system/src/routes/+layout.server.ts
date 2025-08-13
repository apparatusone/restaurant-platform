import { redirect } from '@sveltejs/kit';

// JWT validation without external dependencies
function isJWTExpired(token: string): boolean {
    try {
        const payload = JSON.parse(atob(token.split('.')[1]));
        const currentTime = Math.floor(Date.now() / 1000);
        return payload.exp < currentTime;
    } catch {
        return true; // Invalid token format
    }
}

export const load = async ({ url, cookies, request }) => {
    const isLogin = url.pathname === '/login';
    const isRoot = url.pathname === '/';
    
    // Check authentication - works for both desktop and web/iPad
    const userSession = cookies.get('userSession');
    let user = null;
    
    try {
        user = userSession ? JSON.parse(decodeURIComponent(userSession)) : null;
    } catch (e) {
        // Invalid session data, clear cookie and force re-login
        cookies.set('userSession', '', { path: '/', maxAge: 0, sameSite: 'strict' });
        user = null;
    }
    
    // Additional JWT validation from Authorization header or cookie
    if (user) {
        const authHeader = request.headers.get('authorization');
        let token = null;
        
        if (authHeader && authHeader.startsWith('Bearer ')) {
            token = authHeader.substring(7);
        }
        
        // If we have a token, validate it
        if (token && isJWTExpired(token)) {
            // Token is expired, clear session
            cookies.set('userSession', '', { path: '/', maxAge: 0, sameSite: 'strict' });
            user = null;
        }
    }
    
    // If not authenticated and not on login page, redirect to login
    if (!user && !isLogin) {
        throw redirect(303, '/login');
    }
    
    if (user && isLogin) {
        throw redirect(303, '/dashboard');
    }
    
    // If authenticated and on root, send to placeholder app page
    if (user && isRoot) {
        throw redirect(303, '/dashboard');
    }
    
    return { user };
};