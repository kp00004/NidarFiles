import { useEffect, useRef, useState } from "react";
import Panel from "./Panel";
import { getCoverage, getFrontiers, getMap, getPath } from "../api";
import { renderOccupancyMapCanvas } from "../mapRender";
import type {
  CoverageResponse,
  FrontiersResponse,
  MapResponse,
  PathResponse,
  TelemetryResponse,
} from "../types";

const POLL_INTERVAL_MS = 1000;

// Extension point: /api/map has no real data until onboard-autonomy's
// SLAM/mapping pipeline is running (AUTONOMY_ROADMAP.md Phase 4/5 -- see
// the NIDAR Autonomy Migration report, CHECKPOINT/CURRENT_STATE.md, for
// current status) -- renders an explicit "no map yet" placeholder rather
// than a fake/demo grid, same posture as before this panel had real
// rendering logic. Once /api/map has real data (or the sim/rosbridge_sim
// synthetic feed is used for testing), this renders it for real: occupied/
// free/unknown cells, a coverage overlay, the planned path, the drone's
// current position, and the current frontier/target -- see
// custom-gcs/CLAUDE.md "never invent" for why nothing here is fabricated
// beyond what the backend actually reports.
//
// This is the REAL-mission map, sourced from /api/map (the real /map
// topic) -- for the separate simulation map, see SimulationPanel.tsx,
// which renders via the same renderOccupancyMapCanvas() but from
// /api/simulation/map. The two never share data.
export default function MapPanel({ telemetry }: { telemetry: TelemetryResponse | null }) {
  const [map, setMap] = useState<MapResponse | null>(null);
  const [coverage, setCoverage] = useState<CoverageResponse | null>(null);
  const [path, setPath] = useState<PathResponse | null>(null);
  const [frontiers, setFrontiers] = useState<FrontiersResponse | null>(null);
  const [error, setError] = useState<string | null>(null);
  const canvasRef = useRef<HTMLCanvasElement | null>(null);

  useEffect(() => {
    let mounted = true;

    async function poll() {
      try {
        const [m, c, p, f] = await Promise.all([getMap(), getCoverage(), getPath(), getFrontiers()]);
        if (mounted) {
          setMap(m);
          setCoverage(c);
          setPath(p);
          setFrontiers(f);
          setError(null);
        }
      } catch (e) {
        if (mounted) setError(e instanceof Error ? e.message : String(e));
      }
    }

    poll();
    const id = setInterval(poll, POLL_INTERVAL_MS);
    return () => {
      mounted = false;
      clearInterval(id);
    };
  }, []);

  const hasMap = map?.data != null && map.data.length > 0 && map.width != null && map.height != null;

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas || !hasMap || !map) return;

    renderOccupancyMapCanvas(canvas, {
      map,
      coverage,
      path,
      // Map origin is always (0,0) here: /api/map doesn't carry the
      // origin separately (see MapResponse) -- it's derived from the
      // telemetry mapping block when available, defaulting to (0,0).
      originX: telemetry?.mapping?.origin_x ?? 0,
      originY: telemetry?.mapping?.origin_y ?? 0,
      dronePosition: telemetry?.pose?.position ?? null,
      target: telemetry?.autonomy?.target ?? telemetry?.navigation?.target ?? null,
      frontiers: frontiers?.points ?? null,
    });
  }, [map, coverage, path, frontiers, telemetry, hasMap]);

  return (
    <Panel title="Map" className="col-span-full">
      {error && <div className="text-bad text-xs mb-2">map fetch failed: {error}</div>}
      {hasMap ? (
        <div>
          <div className="text-xs text-dim mb-2">
            {map!.width} × {map!.height} cells, resolution {map!.resolution} m/cell
            {telemetry?.mapping?.explored_pct != null && ` — ${telemetry.mapping.explored_pct}% searched`}
          </div>
          <div className="overflow-auto border border-border rounded">
            <canvas ref={canvasRef} data-testid="map-canvas" />
          </div>
        </div>
      ) : (
        <div className="text-dim text-xs">
          No map data yet — mapping requires onboard-autonomy's SLAM/exploration stack
          to be running and publishing `/map` (see AUTONOMY_ROADMAP.md Phase 4/5).
        </div>
      )}
    </Panel>
  );
}
