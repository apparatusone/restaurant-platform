export class NavigationStore {
    private _activeSection = $state<string>('tables');

    get activeSection() {
        return this._activeSection;
    }

    setActiveSection(section: string) {
        this._activeSection = section;
    }
}

export const navigationStore = new NavigationStore();