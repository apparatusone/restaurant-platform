<script lang="ts">
    import { authState } from '$lib/stores/auth.svelte';
    import { timeclockState } from '$lib/stores/timeclock.svelte';
    import { goto } from '$app/navigation';
    import { onMount } from 'svelte';

    const user = $derived(authState.currentUser);

    onMount(async () => {
        await timeclockState.refresh();
        if (timeclockState.isClockedIn) {
            await goto('/dashboard');
        }
    });

    async function handleClockIn() {
        await timeclockState.clockIn();
        if (timeclockState.isClockedIn) {
            await goto('/dashboard');
        }
    }

    async function handleSwitchUser() {
        document.cookie = 'userSession=; Max-Age=0; path=/; SameSite=Strict';
        localStorage.removeItem('auth_token');
        localStorage.removeItem('currentUser');
        await goto('/login');
    }
</script>

<main class="min-h-screen flex items-center justify-center bg-gradient-to-br from-indigo-500 to-blue-400 p-6">
    <div class="bg-white p-8 rounded-xl shadow-lg w-full max-w-[22rem] min-h-[18rem]">
        <h1 class="text-3xl font-bold text-center text-gray-800 mb-2">Clock In</h1>
        {#if user}
            <div class="text-center text-gray-600 mb-6">{user.name} ({user.role})</div>
        {/if}

        {#if timeclockState.error}
            <div class="text-red-700 bg-red-50 p-2 rounded-md text-center text-sm mb-4">{timeclockState.error}</div>
        {/if}

        <button
            type="button"
            disabled={timeclockState.loading}
            class="w-full py-3 bg-green-600 hover:bg-green-700 text-white rounded-md font-medium disabled:opacity-70 disabled:cursor-not-allowed"
            onclick={handleClockIn}
        >
            {#if timeclockState.loading}Clocking in…{/if}
            {#if !timeclockState.loading}Clock In{/if}
        </button>

        <button
            type="button"
            class="w-full mt-3 py-3 bg-gray-100 hover:bg-gray-200 text-gray-700 rounded-md font-medium"
            onclick={handleSwitchUser}
        >
            Switch User
        </button>
    </div>
</main>

<style></style>
