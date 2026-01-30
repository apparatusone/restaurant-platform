<script lang="ts">
    import { onMount } from "svelte";
    import type { TableWithSession, TableSeating, TableStatus, Staff } from "$lib/types/index.js";
    import FloorView from "../floor/FloorView.svelte";
    import TableSidePanel from "../floor/TableSidePanel.svelte";

    let selectedTable: TableWithSession | undefined = $state();
    let tables: TableWithSession[] = $state([]);
    let seatings: TableSeating[] = $state([]);
    let staff: Staff[] = $state([]);
    let loading = $state(true);
    let error = $state("");

    async function loadFloorData() {
        try {
            loading = true;
            error = "";

            const { tablesApi, tableSeatingsApi } = await import(
                "$lib/api/endpoints/tables.js"
            );
            const [tablesData, sessionsData] = await Promise.all([
                tablesApi.getTablesWithSessions(),
                tableSeatingsApi.getActiveSeatings(),
            ]);

            tables = tablesData;
            seatings = sessionsData;

            // Load staff from API
            const { staffApi } = await import("$lib/api/endpoints/staff.js");
            staff = await staffApi.getAllStaff();
        } catch (err) {
            error =
                err instanceof Error
                    ? err.message
                    : "Failed to load floor data";
        } finally {
            loading = false;
        }
    }

    function getTableStatus(table: TableWithSession): TableStatus {
        if (!table.current_seating_id) return "available";
        const seating = seatings.find((s) => s.id === table.current_seating_id);
        if (!seating || seating.closed_at) return "available";
        return "occupied";
    }

    function selectTable(table: TableWithSession) {
        selectedTable = table;
    }

    function closePanel() {
        selectedTable = undefined;
    }

    async function createSeating(
        tableId: number,
        isReservation: boolean,
        customerName?: string,
        assignedServerId?: number,
    ) {
        try {
            const { tableSeatingsApi } = await import(
                "$lib/api/endpoints/tables.js"
            );
            const newSession = await tableSeatingsApi.createSeating({
                table_id: tableId,
                assigned_server_id: assignedServerId,
                notes: customerName ? `Customer: ${customerName}` : undefined,
            });

            // Update local state
            seatings = [...seatings, newSession];
            tables = tables.map((t) =>
                t.id === tableId
                    ? { ...t, current_seating_id: newSession.id }
                    : t,
            );

            if (selectedTable?.id === tableId) {
                selectedTable = {
                    ...selectedTable,
                    current_seating_id: newSession.id,
                };
            }
        } catch (err) {
            console.error("Failed to create session:", err);
            error = "Failed to create session";
        }
    }

    async function updateSession(
        seatingId: number,
        updates: Partial<TableSeating>,
    ) {
        try {
            const { tableSeatingsApi } = await import(
                "$lib/api/endpoints/tables.js"
            );
            const updatedSession = await tableSeatingsApi.updateSeating(seatingId, updates);
            seatings = seatings.map((s) =>
                s.id === seatingId ? updatedSession : s,
            );
        } catch (err) {
            console.error("Failed to update session:", err);
            error = "Failed to update session";
        }
    }

    async function closeSeating(seatingId: number) {
        try {
            // Check if there are any open checks for this session
            const { checksApi } = await import("$lib/api/endpoints/checks.js");
            const checks = await checksApi.getChecksBySeating(seatingId);
            const openChecks = checks.filter((c: any) => c.status !== "closed");

            if (openChecks.length > 0) {
                error = `Cannot close table: ${openChecks.length} open check(s) must be closed first`;
                return;
            }

            const { tableSeatingsApi } = await import(
                "$lib/api/endpoints/tables.js"
            );
            const closedSession =
                await tableSeatingsApi.closeSeating(seatingId);

            // Update local state
            seatings = seatings.map((s) =>
                s.id === seatingId ? closedSession : s,
            );
            tables = tables.map((t) =>
                t.current_seating_id === seatingId
                    ? { ...t, current_seating_id: undefined }
                    : t,
            );

            if (selectedTable?.current_seating_id === seatingId) {
                selectedTable = { ...selectedTable, current_seating_id: undefined };
            }
        } catch (err) {
            console.error("Failed to close session:", err);
            error =
                err instanceof Error ? err.message : "Failed to close session";
        }
    }

    onMount(loadFloorData);

    let tablesBySection = $derived.by(() => {
        return tables.reduce((acc: Record<string, TableWithSession[]>, table: TableWithSession) => {
            const section = table.section || "Main Floor";
            if (!acc[section]) acc[section] = [];
            acc[section].push(table);
            return acc;
        }, {});
    });

    let currentSeating = $derived.by(() => {
        const seatingId = selectedTable?.current_seating_id;
        if (!seatingId) return undefined;
        return seatings.find((s) => s.id === seatingId && !s.closed_at);
    });
</script>

<div class="flex flex-grow">
    <FloorView
        {loading}
        {error}
        {tablesBySection}
        {selectedTable}
        {getTableStatus}
        onTableSelect={selectTable}
        onRefresh={loadFloorData}
    />
    <div class="w-[50%] lg:w-[40rem]">
        <TableSidePanel
            {selectedTable}
            {currentSeating}
            {staff}
            tableStatus={selectedTable
                ? getTableStatus(selectedTable)
                : undefined}
            onClose={closePanel}
            onCreateSeating={createSeating}
            onUpdateSeating={updateSession}
            onCloseSeating={closeSeating}
        />
    </div>
</div>
