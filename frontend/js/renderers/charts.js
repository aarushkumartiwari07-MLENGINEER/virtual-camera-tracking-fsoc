/**
 * Real-Time Canvas Chart Renderers for Tracking Performance.
 * 1. ErrorChartRenderer: Multi-series Centroid Error vs Time (Raw in Amber, Kalman in Cyan).
 * 2. TimelineChartRenderer: Color-coded Tracking State History Ribbon.
 */

export class ErrorChartRenderer {
  constructor(canvasId) {
    this.canvas = document.getElementById(canvasId);
    this.ctx = this.canvas.getContext('2d');
    this.history = [];
    this.maxPoints = 200;
  }

  addPoint(timestamp, rawErr, kfErr) {
    this.history.push({
      t: timestamp,
      raw: rawErr !== null && rawErr !== undefined ? rawErr : null,
      kf: kfErr !== null && kfErr !== undefined ? kfErr : null,
    });
    if (this.history.length > this.maxPoints) {
      this.history.shift();
    }
  }

  clear() {
    this.history = [];
    this.render();
  }

  render() {
    const ctx = this.ctx;
    const width = this.canvas.width;
    const height = this.canvas.height;
    const padLeft = 35;
    const padRight = 10;
    const padTop = 15;
    const padBottom = 20;

    // 1. Clear Canvas
    ctx.fillStyle = '#05080c';
    ctx.fillRect(0, 0, width, height);

    const plotW = width - padLeft - padRight;
    const plotH = height - padTop - padBottom;

    // 2. Determine Max Y Scale
    let maxVal = 2.0; // Minimum default 2.0 px scale
    for (const pt of this.history) {
      if (pt.raw !== null && pt.raw > maxVal) maxVal = pt.raw;
      if (pt.kf !== null && pt.kf > maxVal) maxVal = pt.kf;
    }
    maxVal = Math.ceil(maxVal * 1.2); // 20% headroom

    // 3. Draw Y Gridlines & Labels
    ctx.strokeStyle = '#182230';
    ctx.lineWidth = 1;
    ctx.fillStyle = '#64748b';
    ctx.font = '9px JetBrains Mono, monospace';
    ctx.textAlign = 'right';

    const yDivs = 4;
    for (let i = 0; i <= yDivs; i++) {
      const val = (maxVal / yDivs) * i;
      const y = padTop + plotH - (val / maxVal) * plotH;

      ctx.beginPath();
      ctx.moveTo(padLeft, y);
      ctx.lineTo(width - padRight, y);
      ctx.stroke();

      ctx.fillText(`${val.toFixed(1)}`, padLeft - 4, y + 3);
    }
    ctx.textAlign = 'left';

    // 4. Draw X Axis Baseline
    ctx.strokeStyle = '#334155';
    ctx.beginPath();
    ctx.moveTo(padLeft, padTop + plotH);
    ctx.lineTo(width - padRight, padTop + plotH);
    ctx.stroke();

    if (this.history.length < 2) {
      ctx.fillStyle = '#475569';
      ctx.fillText('Waiting for telemetry...', width / 2 - 50, height / 2);
      return;
    }

    const n = this.history.length;
    const stepX = plotW / (this.maxPoints - 1);

    // 5. Draw Raw Error Series (Amber)
    ctx.strokeStyle = '#f59e0b';
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    let rawStarted = false;

    for (let i = 0; i < n; i++) {
      const pt = this.history[i];
      if (pt.raw !== null) {
        const x = padLeft + (this.maxPoints - n + i) * stepX;
        const y = padTop + plotH - (Math.min(maxVal, pt.raw) / maxVal) * plotH;
        if (!rawStarted) {
          ctx.moveTo(x, y);
          rawStarted = true;
        } else {
          ctx.lineTo(x, y);
        }
      }
    }
    if (rawStarted) ctx.stroke();

    // 6. Draw Kalman Filtered Error Series (Cyan)
    ctx.strokeStyle = '#38bdf8';
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    let kfStarted = false;

    for (let i = 0; i < n; i++) {
      const pt = this.history[i];
      if (pt.kf !== null) {
        const x = padLeft + (this.maxPoints - n + i) * stepX;
        const y = padTop + plotH - (Math.min(maxVal, pt.kf) / maxVal) * plotH;
        if (!kfStarted) {
          ctx.moveTo(x, y);
          kfStarted = true;
        } else {
          ctx.lineTo(x, y);
        }
      }
    }
    if (kfStarted) ctx.stroke();
  }
}


export class TimelineChartRenderer {
  constructor(canvasId) {
    this.canvas = document.getElementById(canvasId);
    this.ctx = this.canvas.getContext('2d');
    this.history = [];
    this.maxPoints = 300;

    this.stateColors = {
      SEARCHING: '#475569',
      ACQUIRED: '#d97706',
      TRACKING: '#10b981',
      LOST: '#ef4444',
      REACQUIRED: '#38bdf8',
    };
  }

  addState(state) {
    this.history.push(state || 'SEARCHING');
    if (this.history.length > this.maxPoints) {
      this.history.shift();
    }
  }

  clear() {
    this.history = [];
    this.render();
  }

  render() {
    const ctx = this.ctx;
    const width = this.canvas.width;
    const height = this.canvas.height;

    ctx.fillStyle = '#05080c';
    ctx.fillRect(0, 0, width, height);

    if (this.history.length === 0) {
      ctx.fillStyle = '#334155';
      ctx.font = '9px JetBrains Mono, monospace';
      ctx.fillText('No state history recorded.', 10, height / 2 + 3);
      return;
    }

    const n = this.history.length;
    const blockW = width / this.maxPoints;

    for (let i = 0; i < n; i++) {
      const state = this.history[i];
      const color = this.stateColors[state] || '#475569';
      const x = (this.maxPoints - n + i) * blockW;

      ctx.fillStyle = color;
      ctx.fillRect(x, 2, Math.max(1, blockW), height - 4);
    }
  }
}
