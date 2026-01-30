<script lang="ts">
    import {
        ReceiptSolid,
        TableColumnSolid,
    } from "flowbite-svelte-icons";

    interface Props {
        activeSection: string;
        onSectionChange?: (section: string) => void;
        fixed?: boolean; // if true nav is fixed over content, otherwise it participates in layout
    }

    let {
        // active session should be stored
        activeSection = "tables",
        onSectionChange,
        fixed = true,
    }: Props = $props();

    const sections = [
        { id: "tables", label: "Tables", icon: ReceiptSolid },
        { id: "floor", label: "Floor", icon: TableColumnSolid },
        { id: "kitchen", label: "Kitchen", icon: FireSolid },
    ];

    const sizingClass = "min-h-[70px] md:p-4 md:min-h-[80px]";
    const baseButtonClass = `group flex flex-1 flex-col items-center justify-center p-3 text-xs font-medium text-gray-500 hover:bg-gray-50 hover:text-primary-600 dark:text-gray-400 dark:hover:bg-gray-800 dark:hover:text-primary-500 ${sizingClass}`;
    const activeButtonClass =
        "bg-primary-50 text-primary-600 dark:bg-gray-800 dark:text-primary-500";
    const baseIconClass =
        "mb-1 h-8 w-8 group-hover:text-primary-600 dark:group-hover:text-primary-500 md:h-7 md:w-7 text-gray-500 dark:text-gray-400";
    const activeIconClass = "text-primary-600 dark:text-primary-500";

    function handleSectionChange(sectionId: string) {
        onSectionChange?.(sectionId);
        console.log(`Switched to section: ${sectionId}`);
    }
</script>

<nav
    class={fixed
        ? "fixed bottom-0 left-0 right-0 bg-white border-t border-gray-200 shadow-lg z-50"
        : "w-full bg-white border-t border-gray-200 shadow-lg"}
>
    <!-- Desktop/Tablet Layout - All 6 items -->
    <div class="flex max-w-full mx-auto">
        {#each sections as section}
            <button
                class="{baseButtonClass} {activeSection === section.id
                    ? activeButtonClass
                    : ''}"
                onclick={() => handleSectionChange(section.id)}
            >
                <section.icon
                    class="{baseIconClass} {activeSection === section.id
                        ? activeIconClass
                        : ''}"
                />
                <span class="sr-only sm:not-sr-only">{section.label}</span>
            </button>
        {/each}
    </div>

    <!-- Mobile Layout - Only first 3 items -->
    <div class="grid grid-cols-3 max-w-full mx-auto sm:hidden">
        {#each sections.slice(0, 3) as section}
            <button
                class="{baseButtonClass} {activeSection === section.id
                    ? activeButtonClass
                    : ''}"
                onclick={() => handleSectionChange(section.id)}
            >
                <section.icon
                    class="{baseIconClass} {activeSection === section.id
                        ? activeIconClass
                        : ''}"
                />
                <span>{section.label}</span>
            </button>
        {/each}
    </div>
</nav>
