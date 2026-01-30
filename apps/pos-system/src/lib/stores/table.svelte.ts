import type { TableWithSession, TableSeating } from '$lib/types/index.js';

export class TableStore {
    private _tables = $state<TableWithSession[]>([]);
    private _selectedTable = $state<TableWithSession | null>(null);
    private _currentSeating = $state<TableSeating | null>(null);
    private _loading = $state<boolean>(false);
    private _error = $state<string | null>(null);

    // getters
    get tables() { return this._tables; }
    get selectedTable() { return this._selectedTable; }
    get currentSeating() { return this._currentSeating; }
    get loading() { return this._loading; }
    get error() { return this._error; }

    // derived state
    availableTables = $derived(
        this._tables.filter(table => !table.current_seating_id)
    );

    occupiedTables = $derived(
        this._tables.filter(table => table.current_seating_id)
    );

    selectedTableStatus = $derived(() => {
        if (!this._selectedTable) return null;
        return this._selectedTable.current_seating_id ? 'occupied' : 'available';
    });

    // actions
    setTables(tables: TableWithSession[]) {
        this._tables = tables;
    }

    setSelectedTable(table: TableWithSession | null) {
        this._selectedTable = table;
    }

    setCurrentSession(session: TableSeating | null) {
        this._currentSeating = session;
    }

    setLoading(loading: boolean) {
        this._loading = loading;
    }

    setError(error: string | null) {
        this._error = error;
    }

    // update table in list
    updateTable(updatedTable: TableWithSession) {
        this._tables = this._tables.map(table =>
            table.id === updatedTable.id ? updatedTable : table
        );

        // update selected table if it's the same one
        if (this._selectedTable?.id === updatedTable.id) {
            this._selectedTable = updatedTable;
        }
    }

    // find table by id
    findTableById(tableId: number): TableWithSession | undefined {
        return this._tables.find(table => table.id === tableId);
    }

    // clear selection
    clearSelection() {
        this._selectedTable = null;
        this._currentSeating = null;
    }

    // clear all state
    clear() {
        this._tables = [];
        this._selectedTable = null;
        this._currentSeating = null;
        this._loading = false;
        this._error = null;
    }
}

// export singleton instance
export const tableStore = new TableStore();