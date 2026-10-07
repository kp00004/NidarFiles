// Detection-overlay canvas renderer -- mirrors mapRender.ts's "pure
// function draws onto a <canvas> via a ref, given a data bag" pattern.
// Draws raw, unconfirmed image-space person detections on top of the
// live MJPEG <img> in CameraPanel.tsx. Deliberately separate from
// mapRender.ts (different coordinate space -- image pixels, not the
// world/occupancy-grid frame -- and a different data source).
import type { Detection } from "./types";

// Palette matches tailwind.config.js's dark operator-panel colors, same
// rationale as mapRender.ts: canvas fill/stroke styles can't reference
// Tailwind classes.
export const COLOR_BOX = "#d29922"; // warn -- unconfirmed detection, not a confirmed survivor tag
export const COLOR_LABEL_BG = "#d29922";
export const COLOR_LABEL_TEXT = "#0b0f14"; // bg

/** Draws each detection's bbox (scaled from `frameWidth x frameHeight`
 * image-space coordinates to the actually-rendered `displayWidth x
 * displayHeight` size) onto `canvas`, clearing it first. Detections with
 * any null bbox field are skipped rather than crashing -- partial/
 * malformed data is expected from a dev-only model. No-op if the frame
 * size isn't known yet (can't compute a scale factor). */
export function renderDetectionOverlay(
  canvas: HTMLCanvasElement,
  detections: Detection[],
  frameWidth: number,
  frameHeight: number,
  displayWidth: number,
  displayHeight: number,
): void {
  const ctx = canvas.getContext("2d");
  if (!ctx) return;

  ctx.clearRect(0, 0, canvas.width, canvas.height);

  if (!frameWidth || !frameHeight || !displayWidth || !displayHeight) return;

  const scaleX = displayWidth / frameWidth;
  const scaleY = displayHeight / frameHeight;

  for (const detection of detections) {
    const { x_min, y_min, x_max, y_max } = detection.bbox;
    if (x_min == null || y_min == null || x_max == null || y_max == null) continue;

    const x = x_min * scaleX;
    const y = y_min * scaleY;
    const w = (x_max - x_min) * scaleX;
    const h = (y_max - y_min) * scaleY;

    ctx.strokeStyle = COLOR_BOX;
    ctx.lineWidth = 2;
    ctx.strokeRect(x, y, w, h);

    const pct = detection.confidence != null ? Math.round(detection.confidence * 100) : null;
    const label = `person${pct != null ? ` ${pct}%` : ""}`;
    ctx.font = "12px sans-serif";
    const textWidth = ctx.measureText(label).width;
    const labelHeight = 14;

    ctx.fillStyle = COLOR_LABEL_BG;
    ctx.fillRect(x, Math.max(0, y - labelHeight), textWidth + 6, labelHeight);

    ctx.fillStyle = COLOR_LABEL_TEXT;
    ctx.fillText(label, x + 3, Math.max(labelHeight - 3, y - 3));
  }
}
