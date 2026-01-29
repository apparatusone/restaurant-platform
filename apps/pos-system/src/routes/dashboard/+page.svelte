<script lang="ts">
    import DashboardView from '$lib/views/DashboardView.svelte';

    interface Props {
        data: { user: { id: string; name: string; role: string } };
    }
    
    let { data }: Props = $props();
</script>

<DashboardView />
