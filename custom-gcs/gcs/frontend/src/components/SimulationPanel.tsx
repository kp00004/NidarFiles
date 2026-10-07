import { useEffect, useRef, useState } from "react";
import Panel from "./Panel";
import {
  getSimulationCoverage,
  getSimulationMap,
  getSimulationPath,
  getSimulationStatus,
  postSimulationCommand,
} from "../api";
import { renderOccupancyMapCanvas } from "../mapRender";
import type { CoverageResponse, MapResponse, PathResponse, SimulationStatusResponse } from "../types";

const POLL_INTERVAL_MS = 500; // faster than the real MapPanel's -- this is meant to be watched live

// This panel is deliberately styled to look NOTHING like ControlsPanel
// (ok-green START / bad-red ABORT) -- a dashed violet border and an
// explicit "SIMULATION -- NOT REAL FLIGHT" badge on every render, so
// there is no possibility of an operator confusing this with the real
// mission controls. RUN SIMULATION / RESET call postSimulationCommand()
// only, which hits /api/simulation/* -- never postCommand() / the real
// /api/command/* routes (see api.ts, gcs/backend/app/main.py). See
// CHECKPOINT/CURRENT_STATE.md for the full simulation architecture.
export default function SimulationPanel() {
  const [status, setStatus] = useState<SimulationStatusResponse | null>(null);
  const [map, setMap] = useState<MapResponse | null>(null);
  const [coverage, setCoverage] = useState<CoverageResponse | null>(null);
  const [path, setPath] = useState<PathResponse | null>(null);
  const [busy, setBusy] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const canvasRef = useRef<HTMLCanvasElement | null>(null);

  useEffect(() => {
    let mounted = true;

    async function poll() {
      try {
        const [s, m, c, p] = await Promise.all([
          getSimulationStatus(),
          getSimulationMap(),
          getSimulationCoverage(),
          getSimulationPath(),
        ]);
        if (mounted) {
          setStatus(s);
          setMap(m);
          setCoverage(c);
          setPath(p);
        }
      } catch {
        // Polling failure here is non-critical (this is not the real
        // telemetry path) -- silently retry next tick rather than
        // spamming an error the operator can't act on.
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
      dronePosition: status?.pose ?? null,
      target: status?.autonomy?.target ?? null,
    });
  }, [map, coverage, path, status, hasMap]);

  async function send(command: "run" | "reset") {
    if (busy) return;
    setBusy(true);
    setError(null);
    try {
      await postSimulationCommand(command);
    } catch (e) {
      setError(`${command.toUpperCase()} SIMULATION failed: ${e instanceof Error ? e.message : String(e)}`);
    } finally {
      setBusy(false);
    }
  }

  return (
    <Panel title="Simulation" className="col-span-full border-dashed border-2 border-violet-500/60 bg-violet-500/5">
      <div className="inline-block px-2 py-0.5 mb-3 rounded bg-violet-600 text-white text-[11px] font-bold tracking-wide">
        SIMULATION — NOT REAL FLIGHT
      </div>

      <div className="flex gap-2.5 flex-wrap mb-3">
        <button
          type="button"
          className="font-semibold px-4 py-2.5 rounded-md border border-violet-500 bg-violet-600 text-white disabled:opacity-40 disabled:cursor-not-allowed"
          disabled={busy}
          onClick={() => send("run")}
        >
          RUN SIMULATION
        </button>
        <button
          type="button"
          className="font-semibold px-4 py-2.5 rounded-md border border-violet-500 text-violet-300 disabled:opacity-40 disabled:cursor-not-allowed"
          disabled={busy}
          onClick={() => send("reset")}
        >
          RESET
        </button>
      </div>

      {error && <div className="mb-2.5 text-bad text-xs font-semibold">{error}</div>}

      <div className="grid grid-cols-2 sm:grid-cols-4 gap-2 text-xs mb-3">
        <Stat label="Status" value={status?.status ?? "unavailable"} />
        <Stat label="Mission state" value={status?.mission_state ?? "unavailable"} />
        <Stat label="Map explored" value={status ? `${status.map_known_pct}%` : "unavailable"} />
        <Stat label="Coverage searched" value={status ? `${status.coverage_search_pct}%` : "unavailable"} />
        <Stat label="Autonomy state" value={status?.autonomy?.state ?? "unavailable"} />
        <Stat
          label="Target"
          value={status?.autonomy?.target ? `${status.autonomy.target[0].toFixed(2)}, ${status.autonomy.target[1].toFixed(2)}` : "none"}
        />
        <Stat label="Frontiers" value={status?.navigation?.frontier_count ?? "unavailable"} />
        <Stat label="Step" value={status?.step ?? 0} />
      </div>

      {status?.error && <div className="mb-2.5 text-bad text-xs">simulation error: {status.error}</div>}

      {hasMap ? (
        <div className="overflow-auto border border-violet-500/40 rounded">
          <canvas ref={canvasRef} data-testid="simulation-map-canvas" />
        </div>
      ) : (
        <div className="text-dim text-xs">
          No simulation running yet — click RUN SIMULATION to start a deterministic, self-contained
          exploration mission (map, frontier detection, planning, and simulated movement all run for
          real; nothing here touches the real vehicle).
        </div>
      )}
    </Panel>
  );
}

function Stat({ label, value }: { label: string; value: string | number }) {
  return (
    <div className="px-2 py-1.5 rounded bg-black/20 border border-violet-500/20">
      <div className="text-dim text-[10px] uppercase tracking-wide">{label}</div>
      <div className="tabular-nums">{value}</div>
    </div>
  );
}
