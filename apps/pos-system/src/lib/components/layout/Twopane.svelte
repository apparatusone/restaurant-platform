<script lang="ts">
    const { left, right, rightMin = 'lg:min-w-[40rem]' } = $props<{
        left: () => any;
        right: () => any;
        rightMin?: string;
    }>();
</script>

<div class="">
    <!-- Left pane - full width on mobile, flexible on desktop -->
    <div class="">
        {@render left()}
    </div>
    <!-- Right pane - hidden on mobile, fixed width on desktop -->
    <div class={`w-96 border-l border-gray-200`}>
        {@render right()}
    </div>
</div>

<style>
    /* Add any custom styles here */
</style>
