<script lang="ts">
  import DashboardView from '$lib/views/DashboardView.svelte';
  import { authState } from '$lib/stores/auth.svelte';
  import { goto } from '$app/navigation';
  import { onMount } from 'svelte';

  onMount(() => {
    if (!authState.currentUser) {
      goto('/login');
    }
  });
</script>

{#if authState.currentUser}
  <DashboardView />
{:else}
  <div class="loading">Loading...</div>
{/if}

<style>
  .loading {
    display: flex;
    justify-content: center;
    align-items: center;
    height: 100vh;
    font-size: 1.5rem;
    color: #666;
  }
</style>
