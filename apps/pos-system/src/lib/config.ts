// Frontend configuration
// TODO: Load from environment variables or config file

export const config = {
  api: {
    host: 'localhost',
    port: 8000,
    get baseUrl() {
      return `http://${this.host}:${this.port}`;
    }
  },
  get serverUrl() {
    return this.api.baseUrl;
  }
};
