// Shared occupancy-grid canvas renderer -- extracted from MapPanel.tsx so
// SimulationPanel.tsx can render the SAME kind of map (occupancy +
// coverage overlay + planned path + drone position + target) without
// duplicating the drawing logic. Two callers, one renderer -- the
// simulation is a consumer of the same visualization, not a second
// implementation of it.
import type { CoverageResponse, FrontierPointResponse, MapResponse, PathResponse } from "./types";

// Palette matches tailwind.config.js's dark operator-panel colors --
// canvas fills can't reference Tailwind classes, so the same hex values
// are duplicated here deliberately (see tailwind.config.js comment: the
// visual language should stay one system).
export const COLOR_UNKNOWN = "#26313d"; // border
export const COLOR_FREE = "#0b0f14"; // bg
export const COLOR_OCCUPIED = "#e6edf3"; // text
export const COLOR_SEARCHED = "rgba(46, 160, 67, 0.35)"; // ok, translucent
export const COLOR_PATH = "#388bfd"; // accent
export const COLOR_DRONE = "#388bfd"; // accent
export const COLOR_TARGET = "#d29922"; // warn
export const COLOR_FRONTIER = "#a371f7"; // purple, distinct from path/target/drone

// nav_msgs/OccupancyGrid cell semantics (standard for /map and
// /simulation/map; /coverage_grid and /simulation/coverage_grid reuse the
// same encoding with different meaning -- see
// CHECKPOINT/docs/gcs_telemetry_contract.md).
const UNKNOWN = -1;
const OCCUPIED_THRESHOLD = 65;

function worldToCanvas(
  x: number,
  y: number,
  originX: number,
  originY: number,
  resolution: number,
  cellPx: number,
): [number, number] {
  return [((x - originX) / resolution) * cellPx, ((y - originY) / resolution) * cellPx];
}

export interface RenderMapOptions {
  map: MapResponse;
  coverage?: CoverageResponse | null;
  path?: PathResponse | null;
  dronePosition?: { x: number; y: number } | null;
  target?: [number, number] | null;
  frontiers?: FrontierPointResponse[] | null;
  originX?: number;
  originY?: number;
  cellPx?: number;
}

/** Draws one frame of the occupancy map (+ overlays) onto `canvas`. Sizes
 * the canvas to fit the grid at `cellPx` pixels/cell, DPR-aware. Callers
 * are responsible for polling fresh data and calling this again -- this
 * function has no state or timers of its own. */
export function renderOccupancyMapCanvas(canvas: HTMLCanvasElement, options: RenderMapOptions): void {
  const {
    map,
    coverage,
    path,
    dronePosition,
    target,
    frontiers,
    originX = 0,
    originY = 0,
    cellPx = 8,
  } = options;
  if (map.data == null || map.width == null || map.height == null || map.resolution == null) {
    return;
  }

  const width = map.width * cellPx;
  const height = map.height * cellPx;
  const dpr = window.devicePixelRatio || 1;
  canvas.width = width * dpr;
  canvas.height = height * dpr;
  canvas.style.width = `${width}px`;
  canvas.style.height = `${height}px`;

  const ctx = canvas.getContext("2d");
  if (!ctx) return;
  ctx.scale(dpr, dpr);

  // -- occupancy grid --
  for (let row = 0; row < map.height; row++) {
    for (let col = 0; col < map.width; col++) {
      const v = map.data[row * map.width + col];
      ctx.fillStyle = v === UNKNOWN ? COLOR_UNKNOWN : v >= OCCUPIED_THRESHOLD ? COLOR_OCCUPIED : COLOR_FREE;
      // Canvas Y grows downward; occupancy grid Y grows "up" (world
      // convention) -- flip the row so north is up on screen.
      ctx.fillRect(col * cellPx, (map.height - 1 - row) * cellPx, cellPx, cellPx);
    }
  }

  // -- coverage overlay (searched cells only; leaves unsearched/unknown transparent) --
  if (coverage?.data != null && coverage.width != null && coverage.height != null) {
    const covCellPx = (map.width * cellPx) / coverage.width;
    const covCellPy = (map.height * cellPx) / coverage.height;
    for (let row = 0; row < coverage.height; row++) {
      for (let col = 0; col < coverage.width; col++) {
        const v = coverage.data[row * coverage.width + col];
        if (v >= OCCUPIED_THRESHOLD) {
          ctx.fillStyle = COLOR_SEARCHED;
          ctx.fillRect(col * covCellPx, (coverage.height - 1 - row) * covCellPy, covCellPx, covCellPy);
        }
      }
    }
  }

  // -- planned path --
  if (path?.points && path.points.length > 1) {
    ctx.strokeStyle = COLOR_PATH;
    ctx.lineWidth = 2;
    ctx.beginPath();
    path.points.forEach((pt, i) => {
      const [px, py] = worldToCanvas(pt.x, pt.y, originX, originY, map.resolution!, cellPx);
      const canvasY = height - py;
      if (i === 0) ctx.moveTo(px, canvasY);
      else ctx.lineTo(px, canvasY);
    });
    ctx.stroke();
  }

  // -- drone position --
  if (dronePosition) {
    const [dx, dy] = worldToCanvas(dronePosition.x, dronePosition.y, originX, originY, map.resolution, cellPx);
    ctx.fillStyle = COLOR_DRONE;
    ctx.beginPath();
    ctx.arc(dx, height - dy, 4, 0, Math.PI * 2);
    ctx.fill();
    ctx.strokeStyle = "#fff";
    ctx.lineWidth = 1;
    ctx.stroke();
  }

  // -- current frontier/target --
  if (target) {
    const [tx, ty] = worldToCanvas(target[0], target[1], originX, originY, map.resolution, cellPx);
    ctx.strokeStyle = COLOR_TARGET;
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc(tx, height - ty, 6, 0, Math.PI * 2);
    ctx.stroke();
  }

  // -- frontier candidates (exploration boundary points, /frontiers) --
  if (frontiers && frontiers.length > 0) {
    ctx.fillStyle = COLOR_FRONTIER;
    for (const f of frontiers) {
      const [fx, fy] = worldToCanvas(f.x, f.y, originX, originY, map.resolution, cellPx);
      ctx.beginPath();
      ctx.arc(fx, height - fy, 3, 0, Math.PI * 2);
      ctx.fill();
    }
  }
}
