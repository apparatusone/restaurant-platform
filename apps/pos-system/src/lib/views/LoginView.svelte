<script lang="ts">
    import { authState, type User } from '$lib/stores/auth.svelte';
    import { timeclockState } from '$lib/stores/timeclock.svelte';
    import { goto } from '$app/navigation';

    let staffId = $state('');
    let pin = $state('');
    let error = $state('');
    let loading = $state(false);

    const allowedRoles = ['host','server','manager','kitchen','admin'] as const;
    type Role = typeof allowedRoles[number];
    const toRole = (r: string): Role => (allowedRoles as readonly string[]).includes(r) ? (r as Role) : 'host';

    type PinLoginResponse = {
        token: string;
        user: { id: number; name: string; role: string; staff_id: string };
        expires_in: number;
    };

    async function handleLogin(event: Event) {
        event.preventDefault();
        error = '';
        if (!staffId || !pin) {
            error = 'Enter Staff ID and PIN';
            return;
        }

        loading = true;
        try {
            const { apiClient } = await import('$lib/api/client.js');
            const data = await apiClient.request<PinLoginResponse>('auth/pin', {
                method: 'POST',
                body: JSON.stringify({ staff_id: staffId, pin })
            });

            // Persist JWT for API calls
            localStorage.setItem('auth_token', data.token);
            
            apiClient.setAuthToken(data.token);

            const user: User = {
                id: String(data.user.id),
                name: data.user.name,
                role: toRole(data.user.role)
            };

            // Cookie for SSR hydration
            document.cookie = `userSession=${encodeURIComponent(JSON.stringify(user))}; path=/; max-age=86400; SameSite=Strict`;

            authState.login(user);

            await timeclockState.refresh();
            await goto(timeclockState.isClockedIn ? '/dashboard' : '/clock-in');
            pin = '';
        } catch (err) {
            error = err instanceof Error ? err.message : 'Network error. Try again.';
        } finally {
            loading = false;
        }
    }
</script>

<main class="min-h-screen flex items-center justify-center bg-gradient-to-br from-indigo-500 to-blue-400 p-6">
    <div class="bg-white p-8 rounded-xl shadow-lg w-full max-w-[18rem] min-h-[20rem]">
        <h1 class="text-3xl font-bold text-center text-gray-800 mb-6">Employee Login</h1>

        <form class="flex flex-col gap-4" onsubmit={handleLogin} autocomplete="off">
            <input
                type="text"
                inputmode="numeric"
                pattern="[0-9]*"
                autocomplete="one-time-code"
                bind:value={staffId}
                placeholder="Staff ID (4 digits)"
                maxlength="4"
                disabled={loading}
                class="px-4 py-3 border border-gray-300 rounded-md text-sm focus:outline-none focus:ring-2 focus:ring-indigo-400 disabled:opacity-70"
            />

            <input
                type="password"
                inputmode="numeric"
                pattern="[0-9]*"
                autocomplete="one-time-code"
                bind:value={pin}
                placeholder="PIN (4-6 digits)"
                minlength="4"
                maxlength="6"
                disabled={loading}
                class="px-4 py-3 border border-gray-300 rounded-md text-sm focus:outline-none focus:ring-2 focus:ring-indigo-400 disabled:opacity-70"
            />

            {#if error}
                <div class="text-red-700 bg-red-50 p-2 rounded-md text-center text-sm">{error}</div>
            {/if}

            <button type="submit" disabled={loading} class="py-3 bg-indigo-600 hover:bg-indigo-700 text-white rounded-md font-medium disabled:opacity-70 disabled:cursor-not-allowed">
                {#if loading}Logging in…{/if}
                {#if !loading}Login{/if}
            </button>
        </form>
    </div>
</main>

<style></style>
