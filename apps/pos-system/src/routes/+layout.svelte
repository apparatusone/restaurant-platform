<script lang="ts">
	import '../app.css';
	import { afterNavigate, goto } from '$app/navigation';
	import { page } from '$app/state';
	import { authState } from '$lib/stores/auth.svelte';
	import { onMount } from 'svelte';
	
	let { children } = $props();
	let guardInFlight = false;
	let lastRedirectTo: string | null = null;
	let lastRedirectAt = 0;

	async function safeGoto(pathname: string) {
		const now = Date.now();
		if (page.url.pathname === pathname) return;
		if (lastRedirectTo === pathname && now - lastRedirectAt < 1000) return;
		lastRedirectTo = pathname;
		lastRedirectAt = now;
		await goto(pathname);
	}

	async function enforceGuards(pathname: string) {
		if (guardInFlight) return;
		guardInFlight = true;
		try {
		if (pathname === '/login') {
			if (!authState.currentUser) return;
		}

		if (!authState.currentUser) {
			await safeGoto('/login');
			return;
		}

		} finally {
			guardInFlight = false;
		}
	}

	onMount(() => {
		enforceGuards(page.url.pathname);
		afterNavigate(({ to }) => {
			if (!to) return;
			enforceGuards(to.url.pathname);
		});
	});
</script>

{@render children()}
