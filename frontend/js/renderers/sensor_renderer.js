/**
 * Camera Sensor Viewport Canvas 2D Renderer.
 * Renders the simulated 8-bit sensor frame with precision overlays:
 * Optical Principal Point (+), Ground Truth (+), Raw Detected (✕), Kalman Filtered (◯).
 */

export class SensorRenderer {
  constructor(canvasId) {
    this.canvas = document.getElementById(canvasId);
    this.ctx = this.canvas.getContext('2d');
    this.image = new Image();
    this.lastFrameBase64 = null;
  }

  setResolution(width, height) {
    if (this.canvas.width !== width || this.canvas.height !== height) {
      this.canvas.width = width;
      this.canvas.height = height;
    }
  }

  render(telemetry) {
    const ctx = this.ctx;
    const width = this.canvas.width;
    const height = this.canvas.height;
    const u0 = width / 2.0;
    const v0 = height / 2.0;

    // 1. Draw Sensor Image Frame
    if (telemetry && telemetry.sensor_frame_base64) {
      const b64 = telemetry.sensor_frame_base64;
      if (b64 !== this.lastFrameBase64) {
        this.lastFrameBase64 = b64;
        this.image.src = `data:image/jpeg;base64,${b64}`;
      }
      if (this.image.complete && this.image.naturalWidth > 0) {
        ctx.drawImage(this.image, 0, 0, width, height);
      } else {
        ctx.fillStyle = '#05080c';
        ctx.fillRect(0, 0, width, height);
      }
    } else {
      ctx.fillStyle = '#05080c';
      ctx.fillRect(0, 0, width, height);
    }

    // 2. Optical Center Crosshairs (Bore-sight Principal Point)
    ctx.strokeStyle = 'rgba(100, 116, 139, 0.4)';
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(u0, 0); ctx.lineTo(u0, height);
    ctx.moveTo(0, v0); ctx.lineTo(width, v0);
    ctx.stroke();

    // Small center ring
    ctx.strokeStyle = 'rgba(100, 116, 139, 0.6)';
    ctx.beginPath();
    ctx.arc(u0, v0, 8, 0, Math.PI * 2);
    ctx.stroke();

    if (!telemetry) {
      return;
    }

    const gt = telemetry.ground_truth;
    const trk = telemetry.tracking;
    const isVisible = gt && gt.is_visible;

    // 3. Ground Truth Reticle (Green Cross)
    if (isVisible && gt.beacon_image_pos) {
      const gu = gt.beacon_image_pos[0];
      const gv = gt.beacon_image_pos[1];

      ctx.strokeStyle = '#10b981';
      ctx.lineWidth = 1.5;
      const r = 9;

      // Crosshair
      ctx.beginPath();
      ctx.moveTo(gu - r, gv); ctx.lineTo(gu + r, gv);
      ctx.moveTo(gu, gv - r); ctx.lineTo(gu, gv + r);
      ctx.stroke();

      // Corner brackets
      ctx.strokeRect(gu - 6, gv - 6, 12, 12);

      // Coordinate label
      ctx.fillStyle = '#10b981';
      ctx.font = '10px JetBrains Mono, monospace';
      ctx.fillText(`GT (${gu.toFixed(1)}, ${gv.toFixed(1)})`, gu + 12, gv - 8);
    }

    // 4. Raw Classical CV Detected Centroid (Amber ✕ Cross & BBox)
    if (trk && trk.is_detected && trk.raw_centroid) {
      const ru = trk.raw_centroid[0];
      const rv = trk.raw_centroid[1];

      // Draw bounding box if present
      if (trk.raw_bbox) {
        const [umin, vmin, umax, vmax] = trk.raw_bbox;
        ctx.strokeStyle = 'rgba(245, 158, 11, 0.5)';
        ctx.lineWidth = 1;
        ctx.strokeRect(umin, vmin, umax - umin, vmax - vmin);
      }

      // Draw ✕ Cross
      ctx.strokeStyle = '#f59e0b';
      ctx.lineWidth = 2;
      const xSize = 6;
      ctx.beginPath();
      ctx.moveTo(ru - xSize, rv - xSize); ctx.lineTo(ru + xSize, rv + xSize);
      ctx.moveTo(ru - xSize, rv + xSize); ctx.lineTo(ru + xSize, rv - xSize);
      ctx.stroke();

      // Label
      ctx.fillStyle = '#f59e0b';
      ctx.font = '10px JetBrains Mono, monospace';
      ctx.fillText(`RAW (${ru.toFixed(1)}, ${rv.toFixed(1)})`, ru + 12, rv + 6);
    }

    // 5. Kalman Filtered Position (Cyan Circle ◯ & Velocity Vector Arrow)
    if (trk && trk.filtered_centroid) {
      const fu = trk.filtered_centroid[0];
      const fv = trk.filtered_centroid[1];

      // Filtered Circle
      ctx.strokeStyle = trk.is_predicted ? '#ef4444' : '#38bdf8';
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.arc(fu, fv, 7, 0, Math.PI * 2);
      ctx.stroke();

      // Velocity Arrow
      if (trk.filtered_velocity_px_s) {
        const [vu, vv] = trk.filtered_velocity_px_s;
        const speed = Math.hypot(vu, vv);
        if (speed > 1.0) {
          const arrowLen = Math.min(35, speed * 0.25);
          const endX = fu + (vu / speed) * arrowLen;
          const endY = fv + (vv / speed) * arrowLen;

          ctx.strokeStyle = '#38bdf8';
          ctx.lineWidth = 1.5;
          ctx.beginPath();
          ctx.moveTo(fu, fv);
          ctx.lineTo(endX, endY);
          ctx.stroke();

          // Arrow head
          const angle = Math.atan2(vv, vu);
          const headLen = 4;
          ctx.beginPath();
          ctx.moveTo(endX, endY);
          ctx.lineTo(endX - headLen * Math.cos(angle - Math.PI / 6), endY - headLen * Math.sin(angle - Math.PI / 6));
          ctx.moveTo(endX, endY);
          ctx.lineTo(endX - headLen * Math.cos(angle + Math.PI / 6), endY - headLen * Math.sin(angle + Math.PI / 6));
          ctx.stroke();
        }
      }

      // Label
      ctx.fillStyle = trk.is_predicted ? '#ef4444' : '#38bdf8';
      ctx.font = '10px JetBrains Mono, monospace';
      const kfLabel = trk.is_predicted ? `COAST (${fu.toFixed(1)}, ${fv.toFixed(1)})` : `KF (${fu.toFixed(1)}, ${fv.toFixed(1)})`;
      ctx.fillText(kfLabel, fu + 12, fv + 18);
    }

    // 6. Out of FOV Banner
    if (gt && !gt.is_visible) {
      ctx.fillStyle = 'rgba(239, 68, 68, 0.85)';
      ctx.fillRect(width / 2 - 80, 20, 160, 26);
      ctx.strokeStyle = '#ffffff';
      ctx.lineWidth = 1;
      ctx.strokeRect(width / 2 - 80, 20, 160, 26);

      ctx.fillStyle = '#ffffff';
      ctx.font = 'bold 11px JetBrains Mono, monospace';
      ctx.textAlign = 'center';
      ctx.fillText('⚠ BEACON OUT OF FOV', width / 2, 37);
      ctx.textAlign = 'left';
    }
  }
}
