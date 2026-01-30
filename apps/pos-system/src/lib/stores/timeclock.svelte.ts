import { authState } from '$lib/stores/auth.svelte';

export type TimeclockStatus = {
    clocked_in: boolean;
    clock_in_at: string | null;
};

class TimeclockState {
    status = $state<TimeclockStatus | null>(null);
    loading = $state(false);
    error = $state<string | null>(null);

    get isClockedIn() {
        return this.status?.clocked_in ?? false;
    }

    async refresh() {
        if (!authState.currentUser) {
            this.status = null;
            return;
        }

        this.loading = true;
        this.error = null;
        try {
            const { apiClient } = await import('$lib/api/client.js');
            const data = await apiClient.request<TimeclockStatus>('timeclock/status');
            this.status = data;
        } catch (e) {
            this.error = e instanceof Error ? e.message : String(e);
            this.status = null;
        } finally {
            this.loading = false;
        }
    }

    async clockIn() {
        this.loading = true;
        this.error = null;
        try {
            const { apiClient } = await import('$lib/api/client.js');
            const data = await apiClient.request<TimeclockStatus>('timeclock/clock-in', { method: 'POST' });
            this.status = data;
        } catch (e) {
            this.error = e instanceof Error ? e.message : String(e);
        } finally {
            this.loading = false;
        }
    }

    async clockOut() {
        this.loading = true;
        this.error = null;
        try {
            const { apiClient } = await import('$lib/api/client.js');
            const data = await apiClient.request<TimeclockStatus>('timeclock/clock-out', { method: 'POST' });
            this.status = data;
        } catch (e) {
            this.error = e instanceof Error ? e.message : String(e);
        } finally {
            this.loading = false;
        }
    }
}

export const timeclockState = new TimeclockState();
