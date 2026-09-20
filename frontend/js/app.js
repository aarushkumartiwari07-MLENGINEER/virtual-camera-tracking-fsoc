/**
 * Master Application Controller for SIH26169 Developer Frontend.
 * Ties UI elements, REST API actions, WebSocket telemetry stream, and Canvas renderers.
 */

import { ApiClient } from './api.js';
import { TelemetryWebSocket } from './websocket.js';
import { WorldRenderer } from './renderers/world_renderer.js';
import { SensorRenderer } from './renderers/sensor_renderer.js';
import { ErrorChartRenderer, TimelineChartRenderer } from './renderers/charts.js';

class AppController {
  constructor() {
    this.ws = new TelemetryWebSocket('/ws');
    this.worldRenderer = new WorldRenderer('world-canvas');
    this.sensorRenderer = new SensorRenderer('sensor-canvas');
    this.errorChart = new ErrorChartRenderer('error-chart-canvas');
    this.timelineChart = new TimelineChartRenderer('timeline-chart-canvas');

    this.simStatus = 'STOPPED';
    this.trackState = 'SEARCHING';
    this.lastTelemetry = null;

    this.logFilter = 'ALL';
    this.logs = [];

    this.initDOM();
    this.initWebSocket();
    this.initTabSystem();
    this.loadInitialConfig();
    this.pollBackendHealth();
  }

  // =========================================================================
  // DOM Elements & Event Binding
  // =========================================================================
  initDOM() {
    // Header Control Buttons
    this.btnStart = document.getElementById('btn-start');
    this.btnPause = document.getElementById('btn-pause');
    this.btnResume = document.getElementById('btn-resume');
    this.btnStop = document.getElementById('btn-stop');
    this.btnReset = document.getElementById('btn-reset');
    this.btnStep = document.getElementById('btn-step');
    this.btnClearTrail = document.getElementById('btn-clear-trail');

    this.btnStart.addEventListener('click', () => this.handleStart());
    this.btnPause.addEventListener('click', () => this.handlePause());
    this.btnResume.addEventListener('click', () => this.handleResume());
    this.btnStop.addEventListener('click', () => this.handleStop());
    this.btnReset.addEventListener('click', () => this.handleReset());
    this.btnStep.addEventListener('click', () => this.handleStep());
    this.btnClearTrail.addEventListener('click', () => this.worldRenderer.clearTrail());

    // Badges & Telemetry DOM Elements
    this.badgeBackend = document.getElementById('badge-backend');
    this.badgeWs = document.getElementById('badge-ws');
    this.valSimStatus = document.getElementById('val-sim-status');
    this.valTrackState = document.getElementById('val-track-state');
    this.valActualFps = document.getElementById('val-actual-fps');

    // Telemetry Cards
    this.simClockDisplay = document.getElementById('sim-clock-display');
    this.valGtWorld = document.getElementById('val-gt-world');
    this.valGtVel = document.getElementById('val-gt-vel');
    this.valGtImage = document.getElementById('val-gt-image');
    this.valGtVisible = document.getElementById('val-gt-visible');
    this.valRawCentroid = document.getElementById('val-raw-centroid');
    this.valRawMeta = document.getElementById('val-raw-meta');
    this.valKfPos = document.getElementById('val-kf-pos');
    this.valKfVel = document.getElementById('val-kf-vel');
    this.valBoresightErr = document.getElementById('val-boresight-err');
    this.valBoresightAng = document.getElementById('val-boresight-ang');
    this.valTrackingErrors = document.getElementById('val-tracking-errors');
    this.valPipelineLatency = document.getElementById('val-pipeline-latency');
    this.pillFovStatus = document.getElementById('pill-fov-status');
    this.pillWorldBeacon = document.getElementById('pill-world-beacon');

    // Metrics Summary
    this.mDetRate = document.getElementById('m-det-rate');
    this.mRawRmse = document.getElementById('m-raw-rmse');
    this.mKfRmse = document.getElementById('m-kf-rmse');
    this.mAcqTime = document.getElementById('m-acq-time');
    this.mLossRate = document.getElementById('m-loss-rate');
    this.mPipeFps = document.getElementById('m-pipe-fps');
    this.btnResetMetrics = document.getElementById('btn-reset-metrics');
    this.btnResetMetrics.addEventListener('click', () => this.handleResetMetrics());

    // Config Panel Buttons
    this.btnApplyConfig = document.getElementById('btn-apply-config');
    this.btnResetConfig = document.getElementById('btn-reset-config');
    this.btnRandomizeBeacon = document.getElementById('btn-randomize-beacon');

    this.btnApplyConfig.addEventListener('click', () => this.handleApplyConfig());
    this.btnResetConfig.addEventListener('click', () => this.handleResetConfig());
    this.btnRandomizeBeacon.addEventListener('click', () => this.handleRandomizeBeacon());

    // Benchmark Controls
    this.btnRunBenchmark = document.getElementById('btn-run-benchmark');
    this.bmScenarioSelect = document.getElementById('bm-scenario-select');
    this.bmStatusMsg = document.getElementById('bm-status-msg');
    this.bmTableBody = document.getElementById('bm-table-body');
    this.btnRunBenchmark.addEventListener('click', () => this.handleRunBenchmark());

    // Logs Terminal
    this.logsFeed = document.getElementById('logs-feed');
    this.btnClearLogs = document.getElementById('btn-clear-logs');
    this.btnClearLogs.addEventListener('click', () => {
      this.logs = [];
      this.renderLogs();
    });

    document.querySelectorAll('.log-filter').forEach((btn) => {
      btn.addEventListener('click', (e) => {
        document.querySelectorAll('.log-filter').forEach((b) => b.classList.remove('active'));
        e.target.classList.add('active');
        this.logFilter = e.target.getAttribute('data-filter');
        this.renderLogs();
      });
    });
  }

  // =========================================================================
  // Tab System
  // =========================================================================
  initTabSystem() {
    const tabBtns = document.querySelectorAll('.tab-btn');
    const tabPanes = document.querySelectorAll('.tab-pane');

    tabBtns.forEach((btn) => {
      btn.addEventListener('click', () => {
        const targetTab = btn.getAttribute('data-tab');
        tabBtns.forEach((b) => b.classList.remove('active'));
        tabPanes.forEach((p) => p.classList.remove('active'));

        btn.classList.add('active');
        const activePane = document.getElementById(targetTab);
        if (activePane) activePane.classList.add('active');
      });
    });
  }

  // =========================================================================
  // WebSocket Telemetry Handling
  // =========================================================================
  initWebSocket() {
    this.ws.on('status_change', (data) => {
      this.updateWsBadge(data.status);
    });

    this.ws.on('telemetry', (data) => {
      this.handleTelemetry(data.payload);
    });

    this.ws.on('state_change', (data) => {
      this.updateSimulationState(data.simulation_status);
      this.updateTrackingState(data.tracking_state);
    });

    this.ws.on('event_log', (event) => {
      this.addLog(event);
    });

    this.ws.on('benchmark_result', (data) => {
      this.renderBenchmarkResults(data.results);
      this.bmStatusMsg.innerText = `Completed ${data.results.length} scenario(s).`;
      this.btnRunBenchmark.disabled = false;
    });

    this.ws.connect();
  }

  handleTelemetry(packet) {
    if (!packet) return;
    this.lastTelemetry = packet;

    // 1. Render Canvases
    this.worldRenderer.render(packet);
    this.sensorRenderer.render(packet);

    // 2. Update Charts
    const ts = packet.timestamp;
    const rawErr = packet.errors ? packet.errors.raw_error_px : null;
    const kfErr = packet.errors ? packet.errors.filtered_error_px : null;
    this.errorChart.addPoint(ts, rawErr, kfErr);
    this.errorChart.render();

    const trackState = packet.tracking ? packet.tracking.tracking_state : 'SEARCHING';
    this.timelineChart.addState(trackState);
    this.timelineChart.render();

    // 3. Update FSM Active Node
    this.updateFSMVisualizer(trackState, packet.tracking);

    // 4. Update Numerical Telemetry Matrix
    const gt = packet.ground_truth;
    const trk = packet.tracking;

    this.simClockDisplay.innerText = `T+ ${packet.timestamp.toFixed(3)}s • Frame #${packet.frame_index}`;

    if (gt) {
      this.valGtWorld.innerText = `X: ${gt.beacon_world_pos[0].toFixed(1)}, Y: ${gt.beacon_world_pos[1].toFixed(1)}`;
      this.valGtVel.innerText = `Vel: (${gt.beacon_world_vel[0].toFixed(1)}, ${gt.beacon_world_vel[1].toFixed(1)}) px/s`;
      this.pillWorldBeacon.innerText = `Beacon: (${gt.beacon_world_pos[0].toFixed(1)}, ${gt.beacon_world_pos[1].toFixed(1)})`;

      if (gt.beacon_image_pos) {
        this.valGtImage.innerText = `u: ${gt.beacon_image_pos[0].toFixed(1)}, v: ${gt.beacon_image_pos[1].toFixed(1)}`;
      } else {
        this.valGtImage.innerText = `u: N/A, v: N/A`;
      }
      this.valGtVisible.innerText = `Visibility: ${gt.is_visible ? 'TRUE (IN FOV)' : 'FALSE (OUT FOV)'}`;

      this.pillFovStatus.innerText = gt.is_visible ? 'IN FOV' : 'OUT OF FOV';
      this.pillFovStatus.style.borderColor = gt.is_visible ? 'var(--accent-green)' : 'var(--accent-red)';
      this.pillFovStatus.style.color = gt.is_visible ? 'var(--accent-green)' : 'var(--accent-red)';

      if (gt.boresight_pixel_error) {
        this.valBoresightErr.innerText = `du: ${gt.boresight_pixel_error[0].toFixed(1)}, dv: ${gt.boresight_pixel_error[1].toFixed(1)} px`;
      }
      if (gt.boresight_angular_error_deg) {
        this.valBoresightAng.innerText = `dAz: ${gt.boresight_angular_error_deg[0].toFixed(3)}°, dEl: ${gt.boresight_angular_error_deg[1].toFixed(3)}°`;
      }
    }

    if (trk) {
      if (trk.raw_centroid) {
        this.valRawCentroid.innerText = `u: ${trk.raw_centroid[0].toFixed(1)}, v: ${trk.raw_centroid[1].toFixed(1)}`;
      } else {
        this.valRawCentroid.innerText = `u: N/A, v: N/A`;
      }
      this.valRawMeta.innerText = `Area: ${trk.raw_area || 0} px • Candidates: ${trk.candidate_count}`;

      if (trk.filtered_centroid) {
        this.valKfPos.innerText = `u: ${trk.filtered_centroid[0].toFixed(1)}, v: ${trk.filtered_centroid[1].toFixed(1)}`;
      } else {
        this.valKfPos.innerText = `u: N/A, v: N/A`;
      }

      if (trk.filtered_velocity_px_s) {
        this.valKfVel.innerText = `Est Vel: (${trk.filtered_velocity_px_s[0].toFixed(1)}, ${trk.filtered_velocity_px_s[1].toFixed(1)}) px/s`;
      } else {
        this.valKfVel.innerText = `Est Vel: N/A`;
      }

      const rawErrStr = rawErr !== null ? `${rawErr.toFixed(2)} px` : 'N/A';
      const kfErrStr = kfErr !== null ? `${kfErr.toFixed(2)} px` : 'N/A';
      this.valTrackingErrors.innerText = `Raw: ${rawErrStr} • KF: ${kfErrStr}`;
      this.valPipelineLatency.innerText = `Latency: ${trk.processing_time_ms.toFixed(1)} ms`;
    }

    // Refresh Evaluator metrics periodically
    if (packet.frame_index % 10 === 0) {
      this.fetchMetrics();
    }
  }

  updateFSMVisualizer(state, trk) {
    const states = ['SEARCHING', 'ACQUIRED', 'TRACKING', 'LOST', 'REACQUIRED'];
    states.forEach((s) => {
      const node = document.getElementById(`fsm-node-${s}`);
      if (node) {
        node.className = 'fsm-node';
        if (s === state) {
          node.classList.add(`active-${s}`);
        }
      }
    });

    if (trk) {
      document.getElementById('fsm-hits').innerText = trk.consecutive_detections || 0;
      document.getElementById('fsm-losses').innerText = trk.consecutive_losses || 0;
    }
    this.updateTrackingState(state);
  }

  // =========================================================================
  // State and Badge Updaters
  // =========================================================================
  updateWsBadge(status) {
    const dot = this.badgeWs.querySelector('.status-dot');
    const val = this.badgeWs.querySelector('.badge-val');
    val.innerText = status;

    dot.className = 'status-dot';
    if (status === 'CONNECTED') dot.classList.add('dot-green');
    else if (status === 'CONNECTING' || status === 'RECONNECTING') dot.classList.add('dot-amber');
    else dot.classList.add('dot-red');
  }

  updateBackendBadge(online) {
    const dot = this.badgeBackend.querySelector('.status-dot');
    const val = this.badgeBackend.querySelector('.badge-val');
    val.innerText = online ? 'ONLINE' : 'OFFLINE';

    dot.className = 'status-dot';
    if (online) dot.classList.add('dot-green');
    else dot.classList.add('dot-red');
  }

  updateSimulationState(status) {
    this.simStatus = status || 'STOPPED';
    this.valSimStatus.innerText = this.simStatus;

    const dot = this.badgeSim.querySelector('.status-dot');
    dot.className = 'status-dot';

    if (this.simStatus === 'RUNNING') {
      dot.classList.add('dot-green');
      this.btnStart.style.display = 'none';
      this.btnResume.style.display = 'none';
      this.btnPause.style.display = 'inline-flex';
      this.btnPause.disabled = false;
      this.btnStop.disabled = false;
    } else if (this.simStatus === 'PAUSED') {
      dot.classList.add('dot-amber');
      this.btnStart.style.display = 'none';
      this.btnPause.style.display = 'none';
      this.btnResume.style.display = 'inline-flex';
      this.btnResume.disabled = false;
      this.btnStop.disabled = false;
    } else {
      dot.classList.add('dot-gray');
      this.btnStart.style.display = 'inline-flex';
      this.btnResume.style.display = 'none';
      this.btnPause.style.display = 'none';
      this.btnPause.disabled = true;
      this.btnStop.disabled = true;
    }
  }

  updateTrackingState(state) {
    this.trackState = state || 'SEARCHING';
    this.valTrackState.innerText = this.trackState;

    const dot = this.badgeTracking.querySelector('.status-dot');
    dot.className = 'status-dot';

    if (this.trackState === 'TRACKING') dot.classList.add('dot-green');
    else if (this.trackState === 'ACQUIRED' || this.trackState === 'REACQUIRED') dot.classList.add('dot-cyan');
    else if (this.trackState === 'LOST') dot.classList.add('dot-red');
    else dot.classList.add('dot-gray');
  }

  // =========================================================================
  // Control Handlers
  // =========================================================================
  async handleStart() {
    try {
      await ApiClient.startSimulation();
      this.updateSimulationState('RUNNING');
    } catch (err) {
      this.addLog({ level: 'ERROR', category: 'SIMULATION', message: `Start failed: ${err.message}` });
    }
  }

  async handlePause() {
    try {
      await ApiClient.pauseSimulation();
      this.updateSimulationState('PAUSED');
    } catch (err) {
      this.addLog({ level: 'ERROR', category: 'SIMULATION', message: `Pause failed: ${err.message}` });
    }
  }

  async handleResume() {
    try {
      await ApiClient.resumeSimulation();
      this.updateSimulationState('RUNNING');
    } catch (err) {
      this.addLog({ level: 'ERROR', category: 'SIMULATION', message: `Resume failed: ${err.message}` });
    }
  }

  async handleStop() {
    try {
      await ApiClient.stopSimulation();
      this.updateSimulationState('STOPPED');
    } catch (err) {
      this.addLog({ level: 'ERROR', category: 'SIMULATION', message: `Stop failed: ${err.message}` });
    }
  }

  async handleReset() {
    try {
      const motion = document.getElementById('cfg-beacon-motion').value;
      await ApiClient.resetSimulation(motion);
      this.worldRenderer.clearTrail();
      this.errorChart.clear();
      this.timelineChart.clear();
      this.updateSimulationState('STOPPED');
    } catch (err) {
      this.addLog({ level: 'ERROR', category: 'SIMULATION', message: `Reset failed: ${err.message}` });
    }
  }

  async handleStep() {
    try {
      const res = await ApiClient.stepFrame();
      if (res.telemetry) {
        this.handleTelemetry(res.telemetry);
      }
    } catch (err) {
      this.addLog({ level: 'ERROR', category: 'SIMULATION', message: `Step failed: ${err.message}` });
    }
  }

  async handleResetMetrics() {
    try {
      await ApiClient.resetMetrics();
      this.errorChart.clear();
      this.timelineChart.clear();
      this.fetchMetrics();
    } catch (err) {
      this.addLog({ level: 'ERROR', category: 'EVALUATOR', message: `Reset metrics failed: ${err.message}` });
    }
  }

  // =========================================================================
  // Configuration Handlers
  // =========================================================================
  async loadInitialConfig() {
    try {
      const cfg = await ApiClient.getConfig();
      if (cfg) {
        const sim = cfg.simulation;
        const b = sim.beacon;
        const c = sim.camera;
        const d = cfg.detector;
        const k = cfg.kalman;

        document.getElementById('cfg-beacon-motion').value = b.motion_type;
        document.getElementById('cfg-beacon-shape').value = b.shape;
        document.getElementById('cfg-beacon-size').value = b.size;
        if (b.motion_params && b.motion_params.radius) {
          document.getElementById('cfg-motion-radius').value = b.motion_params.radius;
        }
        if (b.motion_params && b.motion_params.angular_speed_deg_s) {
          document.getElementById('cfg-motion-speed').value = b.motion_params.angular_speed_deg_s;
        }

        const resStr = `${c.resolution[0]}x${c.resolution[1]}`;
        document.getElementById('cfg-cam-res').value = resStr;
        document.getElementById('cfg-cam-fov-h').value = c.fov_deg[0];
        document.getElementById('cfg-cam-fov-v').value = c.fov_deg[1];
        document.getElementById('cfg-cam-pan-speed').value = c.max_pan_speed_deg_s;
        document.getElementById('cfg-cam-tilt-speed').value = c.max_tilt_speed_deg_s;
        document.getElementById('cfg-sim-fps').value = sim.fps;

        document.getElementById('cfg-det-thresh').value = d.intensity_threshold;
        document.getElementById('cfg-kf-q').value = k.process_noise_std;
        document.getElementById('cfg-kf-r').value = k.measurement_noise_std;
        document.getElementById('cfg-fsm-lost').value = cfg.state_machine.max_lost_coasting_frames;

        this.worldRenderer.setWorldDimensions(sim.world.width, sim.world.height);
        this.sensorRenderer.setResolution(c.resolution[0], c.resolution[1]);
      }
    } catch (err) {
      console.warn('Could not load initial config:', err);
    }
  }

  async handleApplyConfig() {
    try {
      const [wStr, hStr] = document.getElementById('cfg-cam-res').value.split('x');
      const camW = parseInt(wStr, 10);
      const camH = parseInt(hStr, 10);

      const configPayload = {
        simulation: {
          fps: parseFloat(document.getElementById('cfg-sim-fps').value),
          world: { width: 2000.0, height: 2000.0 },
          beacon: {
            initial_pos: [1000.0, 1000.0],
            size: parseFloat(document.getElementById('cfg-beacon-size').value),
            shape: document.getElementById('cfg-beacon-shape').value,
            intensity: 255.0,
            motion_type: document.getElementById('cfg-beacon-motion').value,
            motion_params: {
              radius: parseFloat(document.getElementById('cfg-motion-radius').value),
              angular_speed_deg_s: parseFloat(document.getElementById('cfg-motion-speed').value),
              amplitude_x: 200.0,
              amplitude_y: 120.0,
              period_s: 6.0,
              speed: 50.0,
              angle_deg: 30.0,
            },
          },
          camera: {
            resolution: [camW, camH],
            fov_deg: [
              parseFloat(document.getElementById('cfg-cam-fov-h').value),
              parseFloat(document.getElementById('cfg-cam-fov-v').value),
            ],
            initial_pos: [1000.0, 1000.0],
            initial_pan_deg: 0.0,
            initial_tilt_deg: 0.0,
            max_pan_speed_deg_s: parseFloat(document.getElementById('cfg-cam-pan-speed').value),
            max_tilt_speed_deg_s: parseFloat(document.getElementById('cfg-cam-tilt-speed').value),
          },
        },
        detector: {
          intensity_threshold: parseFloat(document.getElementById('cfg-det-thresh').value),
          min_area: 2.0,
          max_area: 1500.0,
          min_aspect_ratio: 0.2,
          max_aspect_ratio: 5.0,
          use_adaptive_threshold: false,
        },
        centroid: {
          method: 'intensity_weighted',
          crop_padding: 2,
        },
        kalman: {
          process_noise_std: parseFloat(document.getElementById('cfg-kf-q').value),
          measurement_noise_std: parseFloat(document.getElementById('cfg-kf-r').value),
          initial_error_std: 10.0,
          initial_velocity_std: 50.0,
        },
        state_machine: {
          consecutive_acquire_frames: 3,
          max_lost_coasting_frames: parseInt(document.getElementById('cfg-fsm-lost').value, 10),
          consecutive_reacquire_frames: 2,
        },
      };

      await ApiClient.updateConfig(configPayload);
      this.worldRenderer.clearTrail();
      this.sensorRenderer.setResolution(camW, camH);
      this.addLog({ level: 'INFO', category: 'CONFIG', message: 'Applied updated configuration.' });
    } catch (err) {
      this.addLog({ level: 'ERROR', category: 'CONFIG', message: `Apply failed: ${err.message}` });
    }
  }

  async handleResetConfig() {
    try {
      await ApiClient.resetConfig();
      await this.loadInitialConfig();
      this.worldRenderer.clearTrail();
      this.addLog({ level: 'INFO', category: 'CONFIG', message: 'Configuration restored to defaults.' });
    } catch (err) {
      this.addLog({ level: 'ERROR', category: 'CONFIG', message: `Reset config failed: ${err.message}` });
    }
  }

  async handleRandomizeBeacon() {
    try {
      const rx = 800 + Math.random() * 400;
      const ry = 800 + Math.random() * 400;
      await ApiClient.resetSimulation();
      this.addLog({ level: 'INFO', category: 'SIMULATION', message: `Randomized beacon position to (${rx.toFixed(0)}, ${ry.toFixed(0)})` });
    } catch (err) {
      console.warn('Randomize failed:', err);
    }
  }

  // =========================================================================
  // Benchmark Runner
  // =========================================================================
  async handleRunBenchmark() {
    const scenarioId = this.bmScenarioSelect.value;
    this.bmStatusMsg.innerText = 'Executing benchmark matrix in background...';
    this.btnRunBenchmark.disabled = true;

    try {
      const res = await ApiClient.runBenchmark(scenarioId);
      if (res.results) {
        this.renderBenchmarkResults(res.results);
        this.bmStatusMsg.innerText = `Completed ${res.results.length} scenario(s).`;
      }
    } catch (err) {
      this.bmStatusMsg.innerText = `Benchmark failed: ${err.message}`;
      this.addLog({ level: 'ERROR', category: 'BENCHMARK', message: `Benchmark run failed: ${err.message}` });
    } finally {
      this.btnRunBenchmark.disabled = false;
    }
  }

  renderBenchmarkResults(results) {
    if (!results || results.length === 0) return;

    this.bmTableBody.innerHTML = results.map((r) => `
      <tr>
        <td><strong>${r.scenario_name}</strong></td>
        <td>${r.motion_type} (${r.shape})</td>
        <td>${r.noise_std_px > 0 ? `${r.noise_std_px} px` : '0 (Clean)'}</td>
        <td>${r.raw_rmse_px.toFixed(3)} px</td>
        <td style="color: var(--accent-cyan); font-weight:700;">${r.filtered_rmse_px.toFixed(3)} px</td>
        <td>${r.raw_max_err_px.toFixed(2)} px</td>
        <td>${r.filtered_max_err_px.toFixed(2)} px</td>
        <td>${r.velocity_rmse_px_s !== null ? `${r.velocity_rmse_px_s.toFixed(2)} px/s` : 'N/A'}</td>
        <td>${r.coasting_max_err_px !== null ? `${r.coasting_max_err_px.toFixed(2)} px` : 'N/A'}</td>
        <td class="text-muted">${r.notes || ''}</td>
      </tr>
    `).join('');
  }

  // =========================================================================
  // Metrics Fetcher
  // =========================================================================
  async fetchMetrics() {
    try {
      const m = await ApiClient.getMetrics();
      if (m) {
        this.mDetRate.innerText = `${m.detection_rate_pct.toFixed(1)}%`;
        this.mRawRmse.innerText = `${m.rmse_raw_error_px.toFixed(3)} px`;
        this.mKfRmse.innerText = `${m.rmse_filtered_error_px.toFixed(3)} px`;
        this.mAcqTime.innerText = m.acquisition_time_s !== null ? `${m.acquisition_time_s.toFixed(3)} s` : 'N/A';
        this.mLossRate.innerText = `${m.target_loss_rate_pct.toFixed(1)}%`;
        this.mPipeFps.innerText = `${Math.round(m.processing_fps)} FPS`;
      }
    } catch (err) {
      // Ignored during shutdown
    }
  }

  // =========================================================================
  // Logging Terminal
  // =========================================================================
  addLog(entry) {
    const timestamp = entry.timestamp ? new Date(entry.timestamp * 1000).toLocaleTimeString() : new Date().toLocaleTimeString();
    const logObj = {
      time: timestamp,
      level: entry.level || 'INFO',
      category: entry.category || 'SYSTEM',
      message: entry.message || '',
    };
    this.logs.push(logObj);
    if (this.logs.length > 200) this.logs.shift();
    this.renderLogs();
  }

  renderLogs() {
    const filtered = this.logFilter === 'ALL'
      ? this.logs
      : this.logs.filter((l) => l.level === this.logFilter);

    this.logsFeed.innerHTML = filtered.map((l) => `
      <div class="log-entry log-${l.level}">
        <span class="log-time">[${l.time}]</span>
        <span class="log-cat">[${l.category}]</span>
        <span class="log-msg">${l.message}</span>
      </div>
    `).join('');

    this.logsFeed.scrollTop = this.logsFeed.scrollHeight;
  }

  // =========================================================================
  // Health Poller
  // =========================================================================
  pollBackendHealth() {
    setInterval(async () => {
      try {
        const status = await ApiClient.getStatus();
        this.updateBackendBadge(true);
        this.valActualFps.innerText = `${status.fps} FPS`;
      } catch (err) {
        this.updateBackendBadge(false);
      }
    }, 2000);
  }
}

// Initialize on page load
window.addEventListener('DOMContentLoaded', () => {
  window.app = new AppController();
});
