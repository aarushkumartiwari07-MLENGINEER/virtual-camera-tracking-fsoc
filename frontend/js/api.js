/**
 * REST API Client for SIH26169 Backend Service.
 */

const API_BASE = '/api';

export class ApiClient {
  static async request(endpoint, options = {}) {
    const url = `${API_BASE}${endpoint}`;
    const defaultHeaders = {
      'Content-Type': 'application/json',
    };

    try {
      const response = await fetch(url, {
        ...options,
        headers: { ...defaultHeaders, ...(options.headers || {}) },
      });

      if (!response.ok) {
        const errorText = await response.text();
        throw new Error(`API ${response.status}: ${errorText || response.statusText}`);
      }

      return await response.json();
    } catch (error) {
      console.error(`[API ERROR] ${endpoint}:`, error);
      throw error;
    }
  }

  // Simulation Controls
  static async getStatus() {
    return this.request('/simulation/status');
  }

  static async startSimulation() {
    return this.request('/simulation/start', { method: 'POST' });
  }

  static async pauseSimulation() {
    return this.request('/simulation/pause', { method: 'POST' });
  }

  static async resumeSimulation() {
    return this.request('/simulation/resume', { method: 'POST' });
  }

  static async stopSimulation() {
    return this.request('/simulation/stop', { method: 'POST' });
  }

  static async resetSimulation(motion = null) {
    const query = motion ? `?motion=${encodeURIComponent(motion)}` : '';
    return this.request(`/simulation/reset${query}`, { method: 'POST' });
  }

  static async stepFrame() {
    return this.request('/simulation/step', { method: 'POST' });
  }

  // Configuration
  static async getConfig() {
    return this.request('/config');
  }

  static async updateConfig(configData) {
    return this.request('/config', {
      method: 'PUT',
      body: JSON.stringify(configData),
    });
  }

  static async resetConfig() {
    return this.request('/config/reset', { method: 'POST' });
  }

  // Metrics
  static async getMetrics() {
    return this.request('/metrics');
  }

  static async getMetricsHistory() {
    return this.request('/metrics/history');
  }

  static async resetMetrics() {
    return this.request('/metrics/reset', { method: 'POST' });
  }

  // Benchmarks
  static async getBenchmarkScenarios() {
    return this.request('/benchmarks/scenarios');
  }

  static async getLatestBenchmarkResults() {
    return this.request('/benchmarks/latest');
  }

  static async runBenchmark(scenarioId = 'all', customParams = {}) {
    return this.request('/benchmarks/run', {
      method: 'POST',
      body: JSON.stringify({
        scenario_id: scenarioId,
        ...customParams,
      }),
    });
  }
}
