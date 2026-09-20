/**
 * WebSocket Connection Client for real-time telemetry streaming.
 * Handles automatic reconnect with exponential backoff and message routing.
 */

export class TelemetryWebSocket {
  constructor(endpoint = '/ws') {
    this.endpoint = endpoint;
    this.socket = null;
    this.listeners = new Map();
    this.reconnectAttempts = 0;
    this.maxReconnectAttempts = 20;
    this.reconnectDelay = 1000;
    this.isConnected = false;
    this.shouldReconnect = true;
  }

  connect() {
    const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    const host = window.location.host;
    const wsUrl = `${protocol}//${host}${this.endpoint}`;

    console.log(`[WS] Connecting to ${wsUrl}...`);
    this.notifyStatus('CONNECTING');

    try {
      this.socket = new WebSocket(wsUrl);

      this.socket.onopen = () => {
        console.log('[WS] Connected successfully.');
        this.isConnected = true;
        this.reconnectAttempts = 0;
        this.reconnectDelay = 1000;
        this.notifyStatus('CONNECTED');
      };

      this.socket.onmessage = (event) => {
        try {
          const data = JSON.parse(event.data);
          this.dispatchEvent(data.type || 'message', data);
        } catch (err) {
          console.warn('[WS] Failed to parse message:', event.data);
        }
      };

      this.socket.onclose = (event) => {
        console.warn(`[WS] Closed (code=${event.code}).`);
        this.isConnected = false;
        this.notifyStatus('DISCONNECTED');
        if (this.shouldReconnect) {
          this.scheduleReconnect();
        }
      };

      this.socket.onerror = (error) => {
        console.error('[WS] Error:', error);
        this.notifyStatus('ERROR');
      };

    } catch (err) {
      console.error('[WS] Connection exception:', err);
      this.scheduleReconnect();
    }
  }

  scheduleReconnect() {
    if (this.reconnectAttempts >= this.maxReconnectAttempts) {
      console.error('[WS] Max reconnection attempts reached.');
      this.notifyStatus('FAILED');
      return;
    }

    this.reconnectAttempts++;
    const delay = Math.min(10000, this.reconnectDelay * Math.pow(1.3, this.reconnectAttempts));
    console.log(`[WS] Reconnecting in ${(delay / 1000).toFixed(1)}s (Attempt ${this.reconnectAttempts})...`);
    this.notifyStatus('RECONNECTING');

    setTimeout(() => {
      this.connect();
    }, delay);
  }

  sendCommand(command, params = {}) {
    if (this.isConnected && this.socket.readyState === WebSocket.OPEN) {
      this.socket.send(JSON.stringify({ command, ...params }));
    } else {
      console.warn('[WS] Cannot send command: socket not open.');
    }
  }

  on(eventType, callback) {
    if (!this.listeners.has(eventType)) {
      this.listeners.set(eventType, []);
    }
    this.listeners.get(eventType).push(callback);
  }

  off(eventType, callback) {
    if (this.listeners.has(eventType)) {
      const filtered = this.listeners.get(eventType).filter((cb) => cb !== callback);
      this.listeners.set(eventType, filtered);
    }
  }

  dispatchEvent(eventType, data) {
    if (this.listeners.has(eventType)) {
      this.listeners.get(eventType).forEach((cb) => cb(data));
    }
    // Also dispatch to wildcard '*' listeners
    if (this.listeners.has('*')) {
      this.listeners.get('*').forEach((cb) => cb(data));
    }
  }

  notifyStatus(status) {
    this.dispatchEvent('status_change', { status });
  }

  close() {
    this.shouldReconnect = false;
    if (this.socket) {
      this.socket.close();
    }
  }
}
