<script lang="ts">
  interface Props {
    variant?: 'default' | 'elevated' | 'outlined';
    padding?: 'sm' | 'md' | 'lg';
    clickable?: boolean;
    selected?: boolean;
    disabled?: boolean;
    onclick?: () => void;
    children: import('svelte').Snippet;
  }

  let { 
    variant = 'default',
    padding = 'md',
    clickable = false,
    selected = false,
    disabled = false,
    onclick,
    children
  }: Props = $props();

  const baseClasses = 'rounded-lg transition-all duration-200';
  const variantClasses = {
    default: 'bg-white border border-gray-200',
    elevated: 'bg-white shadow-md hover:shadow-lg',
    outlined: 'bg-transparent border-2 border-gray-300'
  };
  const paddingClasses = {
    sm: 'p-3',
    md: 'p-4',
    lg: 'p-6'
  };

  function handleClick() {
    if (!disabled && onclick) {
      onclick();
    }
  }

  function handleKeydown(event: KeyboardEvent) {
    if (event.key === 'Enter' || event.key === ' ') {
      event.preventDefault();
      handleClick();
    }
  }
</script>

{#if clickable}
  <button
    type="button"
    class="{baseClasses} {variantClasses[variant]} {paddingClasses[padding]} w-full text-left"
    class:cursor-not-allowed={disabled}
    class:opacity-50={disabled}
    class:ring-2={selected}
    class:ring-blue-500={selected}
    class:hover:shadow-md={!disabled && variant === 'default'}
    disabled={disabled}
    onclick={handleClick}
  >
    {@render children()}
  </button>
{:else}
  <div
    class="{baseClasses} {variantClasses[variant]} {paddingClasses[padding]}"
  >
    {@render children()}
  </div>
{/if}