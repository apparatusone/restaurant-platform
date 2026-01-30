<script lang="ts">
    import { SortHorizontalOutline } from "flowbite-svelte-icons";

    export let user: { id: string; name: string; role: string };

    function handleSwitchUser() {
        console.log('Switch user clicked');
        document.cookie = 'userSession=; Max-Age=0; path=/; SameSite=Strict';
        localStorage.removeItem('auth_token');
        localStorage.removeItem('currentUser');
        window.location.href = '/login';
    }
</script>

<header class="flex justify-between items-center p-2 md:p-3 bg-white border-b border-gray-200 shadow-sm min-h-[50px] md:min-h-[50px] z-50">
    <div class="flex gap-2 items-center"></div>

    <!-- Center: User name -->
    <div class="flex flex-col items-center text-center flex-1">
        <span class="text-sm md:text-base font-semibold text-gray-700 leading-tight">{user.name}</span>
        <span class="text-xs md:text-sm font-medium text-gray-500">({user.role})</span>
    </div>

    <!-- Right side: Switch user -->
    <div class="flex gap-2 items-center">
        <button class="flex flex-col items-center justify-center p-1.5 md:p-2 bg-transparent border-none cursor-pointer transition-all duration-200 rounded-md min-w-[60px] md:min-w-[70px] min-h-[50px] md:min-h-[50px] hover:bg-gray-100" on:click={handleSwitchUser}>
            <SortHorizontalOutline class="w-4 h-4 md:w-5 md:h-5 mb-1 text-gray-500 transition-colors duration-200" />
            <span class="text-xs md:text-sm font-medium text-gray-500">Switch User</span>
        </button>
    </div>
</header>

<style></style>