/**
 * World View Canvas 2D Renderer.
 * Renders the 2000x2000 virtual search space, coordinate grid, camera position,
 * dynamic FOV frustum footprint, and beacon trajectory trail.
 */

export class WorldRenderer {
  constructor(canvasId) {
    this.canvas = document.getElementById(canvasId);
    this.ctx = this.canvas.getContext('2d');
    this.worldWidth = 2000.0;
    this.worldHeight = 2000.0;
    this.trail = [];
    this.maxTrail = 180;
  }

  setWorldDimensions(width, height) {
    this.worldWidth = width || 2000.0;
    this.worldHeight = height || 2000.0;
  }

  clearTrail() {
    this.trail = [];
    this.render({});
  }

  worldToCanvas(wx, wy) {
    const scaleX = this.canvas.width / this.worldWidth;
    const scaleY = this.canvas.height / this.worldHeight;
    return {
      x: wx * scaleX,
      y: wy * scaleY,
    };
  }

  render(telemetry) {
    const ctx = this.ctx;
    const width = this.canvas.width;
    const height = this.canvas.height;

    // 1. Clear Background
    ctx.fillStyle = '#05080c';
    ctx.fillRect(0, 0, width, height);

    // 2. Draw Coordinate Grid (5x5 or 10x10 subdivisions)
    ctx.strokeStyle = '#16202d';
    ctx.lineWidth = 1;
    const gridDivs = 4;
    for (let i = 1; i < gridDivs; i++) {
      const gx = (width / gridDivs) * i;
      const gy = (height / gridDivs) * i;
      ctx.beginPath();
      ctx.moveTo(gx, 0);
      ctx.lineTo(gx, height);
      ctx.stroke();

      ctx.beginPath();
      ctx.moveTo(0, gy);
      ctx.lineTo(width, gy);
      ctx.stroke();

      // Grid coordinate labels
      ctx.fillStyle = '#334155';
      ctx.font = '9px JetBrains Mono, monospace';
      ctx.fillText(`${(this.worldWidth / gridDivs) * i}`, gx + 4, 12);
      ctx.fillText(`${(this.worldHeight / gridDivs) * i}`, 4, gy - 4);
    }

    if (!telemetry || !telemetry.ground_truth) {
      return;
    }

    const gt = telemetry.ground_truth;
    const bPos = gt.beacon_world_pos;
    const bVel = gt.beacon_world_vel;
    const cPos = gt.camera_world_pos;
    const panDeg = gt.camera_pan_deg;
    const tiltDeg = gt.camera_tilt_deg;
    const fovDeg = gt.fov_deg;

    // 3. Update & Draw Beacon Trail
    if (bPos) {
      this.trail.push({ x: bPos[0], y: bPos[1] });
      if (this.trail.length > this.maxTrail) {
        this.trail.shift();
      }

      if (this.trail.length > 1) {
        ctx.beginPath();
        for (let i = 0; i < this.trail.length; i++) {
          const pt = this.worldToCanvas(this.trail[i].x, this.trail[i].y);
          if (i === 0) ctx.moveTo(pt.x, pt.y);
          else ctx.lineTo(pt.x, pt.y);
        }
        ctx.strokeStyle = 'rgba(16, 185, 129, 0.4)';
        ctx.lineWidth = 2;
        ctx.stroke();
      }
    }

    // 4. Draw Camera Dynamic FOV Frustum
    // FOV footprint in angular search plane:
    // Scale = 4°/640 = 0.00625°/px. Footprint width = FOV_h / Scale_x = 640.
    // Center of FOV in world: (X_cam + pan/Scale_x, Y_cam + tilt/Scale_y)
    const scaleX = fovDeg[0] / 640.0;
    const scaleY = fovDeg[1] / 480.0;
    const fovWidthWorld = fovDeg[0] / (scaleX || 0.00625);
    const fovHeightWorld = fovDeg[1] / (scaleY || 0.00625);

    const fovCenterWorldX = cPos[0] + panDeg / (scaleX || 0.00625);
    const fovCenterWorldY = cPos[1] + tiltDeg / (scaleY || 0.00625);

    const fovMinWorldX = fovCenterWorldX - fovWidthWorld / 2.0;
    const fovMinWorldY = fovCenterWorldY - fovHeightWorld / 2.0;

    const fovMinCanvas = this.worldToCanvas(fovMinWorldX, fovMinWorldY);
    const fovMaxCanvas = this.worldToCanvas(fovMinWorldX + fovWidthWorld, fovMinWorldY + fovHeightWorld);
    const fovWCanvas = fovMaxCanvas.x - fovMinCanvas.x;
    const fovHCanvas = fovMaxCanvas.y - fovMinCanvas.y;

    // Fill FOV footprint
    const isVisible = gt.is_visible;
    ctx.fillStyle = isVisible ? 'rgba(56, 189, 248, 0.08)' : 'rgba(239, 68, 68, 0.06)';
    ctx.fillRect(fovMinCanvas.x, fovMinCanvas.y, fovWCanvas, fovHCanvas);

    // Border FOV footprint
    ctx.strokeStyle = isVisible ? '#38bdf8' : '#ef4444';
    ctx.lineWidth = 1.5;
    ctx.strokeRect(fovMinCanvas.x, fovMinCanvas.y, fovWCanvas, fovHCanvas);

    // FOV Frustum Lines from Camera Position to Frustum Corners
    const camCanvas = this.worldToCanvas(cPos[0], cPos[1]);
    ctx.strokeStyle = 'rgba(56, 189, 248, 0.3)';
    ctx.lineWidth = 1;
    ctx.setLineDash([3, 3]);
    ctx.beginPath();
    ctx.moveTo(camCanvas.x, camCanvas.y); ctx.lineTo(fovMinCanvas.x, fovMinCanvas.y);
    ctx.moveTo(camCanvas.x, camCanvas.y); ctx.lineTo(fovMinCanvas.x + fovWCanvas, fovMinCanvas.y);
    ctx.moveTo(camCanvas.x, camCanvas.y); ctx.lineTo(fovMinCanvas.x + fovWCanvas, fovMinCanvas.y + fovHCanvas);
    ctx.moveTo(camCanvas.x, camCanvas.y); ctx.lineTo(fovMinCanvas.x, fovMinCanvas.y + fovHCanvas);
    ctx.stroke();
    ctx.setLineDash([]);

    // 5. Draw Camera Center (Cyan Diamond)
    ctx.fillStyle = '#38bdf8';
    ctx.beginPath();
    ctx.moveTo(camCanvas.x, camCanvas.y - 6);
    ctx.lineTo(camCanvas.x + 6, camCanvas.y);
    ctx.lineTo(camCanvas.x, camCanvas.y + 6);
    ctx.lineTo(camCanvas.x - 6, camCanvas.y);
    ctx.closePath();
    ctx.fill();

    // 6. Draw Beacon (Bright Green Circle) & Velocity Vector
    if (bPos) {
      const bCanvas = this.worldToCanvas(bPos[0], bPos[1]);

      // Velocity arrow
      if (bVel && (Math.abs(bVel[0]) > 0.1 || Math.abs(bVel[1]) > 0.1)) {
        const velLen = Math.hypot(bVel[0], bVel[1]);
        const arrowLen = Math.min(25, velLen * 0.3);
        const vxNorm = bVel[0] / (velLen || 1);
        const vyNorm = bVel[1] / (velLen || 1);
        const arrowEnd = {
          x: bCanvas.x + vxNorm * arrowLen,
          y: bCanvas.y + vyNorm * arrowLen,
        };

        ctx.strokeStyle = '#f59e0b';
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(bCanvas.x, bCanvas.y);
        ctx.lineTo(arrowEnd.x, arrowEnd.y);
        ctx.stroke();
      }

      // Beacon Dot
      ctx.fillStyle = '#10b981';
      ctx.beginPath();
      ctx.arc(bCanvas.x, bCanvas.y, 5, 0, Math.PI * 2);
      ctx.fill();
      ctx.strokeStyle = '#ffffff';
      ctx.lineWidth = 1;
      ctx.stroke();
    }
  }
}
