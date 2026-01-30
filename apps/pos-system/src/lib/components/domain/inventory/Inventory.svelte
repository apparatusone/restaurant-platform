<script lang="ts">
    import { onMount } from 'svelte';
    import { ingredientsApi, type Ingredient } from '$lib/api/endpoints/ingredients.js';

    interface Props {
        user?: any;
    }

    let { user }: Props = $props();

    let ingredients = $state<Ingredient[]>([]);
    let loading = $state(false);
    let error = $state<string | null>(null);
    let editingId = $state<number | null>(null);
    let editAmount = $state(0);
    let editApriltagId = $state<number | string | null>(null);

    onMount(() => {
        loadIngredients();
    });

    async function loadIngredients() {
        try {
            loading = true;
            error = null;
            ingredients = await ingredientsApi.getAll();
        } catch (err) {
            error = 'Failed to load ingredients';
            console.error(err);
        } finally {
            loading = false;
        }
    }

    function startEdit(ingredient: Ingredient) {
        editingId = ingredient.id;
        editAmount = ingredient.quantity_on_hand;
        editApriltagId = ingredient.apriltag_id ?? '';
    }

    function cancelEdit() {
        editingId = null;
    }

    async function saveEdit(id: number) {
        try {
            const apriltagValue = editApriltagId === '' || editApriltagId === null 
                ? null 
                : Number(editApriltagId);
            
            await ingredientsApi.update(id, {
                quantity_on_hand: editAmount,
                apriltag_id: apriltagValue
            });
            editingId = null;
            await loadIngredients();
        } catch (err) {
            error = 'Failed to update ingredient';
            console.error(err);
        }
    }

    async function removeApriltagId(id: number) {
        try {
            await ingredientsApi.update(id, {
                apriltag_id: null
            });
            await loadIngredients();
        } catch (err) {
            error = 'Failed to remove AprilTag ID';
            console.error(err);
        }
    }
</script>

<div class="inventory-view">
    <div class="header">
        <h1>Inventory</h1>
        <button onclick={loadIngredients} disabled={loading}>
            {loading ? 'Loading...' : 'Refresh'}
        </button>
    </div>

    {#if error}
        <div class="error">{error}</div>
    {/if}

    <table>
        <thead>
            <tr>
                <th>Item</th>
                <th>Stock</th>
                <th>AprilTag ID</th>
                <th>Actions</th>
            </tr>
        </thead>
        <tbody>
            {#each ingredients as ingredient (ingredient.id)}
                <tr>
                    <td>{ingredient.name}</td>
                    <td>
                        {#if editingId === ingredient.id}
                            <input type="number" bind:value={editAmount} min="0" />
                        {:else}
                            {ingredient.quantity_on_hand}
                        {/if}
                    </td>
                    <td>
                        {#if editingId === ingredient.id}
                            <input 
                                type="text" 
                                bind:value={editApriltagId} 
                                placeholder="None"
                                pattern="[0-9]*"
                            />
                        {:else}
                            <div class="apriltag-cell">
                                <span class="apriltag-value">{ingredient.apriltag_id ?? '—'}</span>
                                {#if ingredient.apriltag_id !== null}
                                    <button class="remove-tag" onclick={() => removeApriltagId(ingredient.id)}>✕</button>
                                {/if}
                            </div>
                        {/if}
                    </td>
                    <td>
                        {#if editingId === ingredient.id}
                            <button class="save" onclick={() => saveEdit(ingredient.id)}>Save</button>
                            <button class="cancel" onclick={cancelEdit}>Cancel</button>
                        {:else}
                            <button onclick={() => startEdit(ingredient)}>Edit</button>
                        {/if}
                    </td>
                </tr>
            {/each}
        </tbody>
    </table>
</div>

<style>
    .inventory-view {
        padding: 1rem;
        width: 100%;
    }

    .header {
        display: flex;
        justify-content: space-between;
        align-items: center;
        margin-bottom: 1rem;
    }

    .header h1 {
        margin: 0;
        font-size: 1.5rem;
    }

    .header button {
        padding: 0.5rem 1rem;
        background: #007bff;
        color: white;
        border: none;
        border-radius: 4px;
        cursor: pointer;
    }

    .header button:disabled {
        background: #6c757d;
    }

    .error {
        background: #f8d7da;
        color: #721c24;
        padding: 0.75rem;
        border-radius: 4px;
        margin-bottom: 1rem;
    }

    table {
        width: 100%;
        border-collapse: collapse;
        background: white;
        border-radius: 8px;
        overflow: hidden;
        box-shadow: 0 1px 3px rgba(0,0,0,0.1);
    }

    th, td {
        padding: 0.75rem 1rem;
        text-align: left;
        border-bottom: 1px solid #e9ecef;
    }

    th {
        background: #f8f9fa;
        font-weight: 600;
    }

    input[type="number"] {
        width: 80px;
        padding: 0.25rem 0.5rem;
        border: 1px solid #ced4da;
        border-radius: 4px;
    }

    td button {
        padding: 0.25rem 0.75rem;
        border: none;
        border-radius: 4px;
        cursor: pointer;
        margin-right: 0.25rem;
    }

    td button:not(.save):not(.cancel):not(.remove-tag) {
        background: #e9ecef;
    }

    .save {
        background: #28a745;
        color: white;
    }

    .cancel {
        background: #dc3545;
        color: white;
    }

    .remove-tag {
        background: #dc3545 !important;
        color: white !important;
        padding: 0.125rem 0.375rem;
        font-size: 0.875rem;
        margin-left: 0.5rem;
    }

    .apriltag-cell {
        display: flex;
        align-items: center;
        gap: 0.5rem;
    }

    .apriltag-value {
        min-width: 2rem;
    }
</style>
