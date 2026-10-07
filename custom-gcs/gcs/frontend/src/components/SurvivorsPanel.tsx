import { useEffect, useState } from "react";
import Panel from "./Panel";
import { getSurvivors } from "../api";
import type { SurvivorResponse } from "../types";

const POLL_INTERVAL_MS = 1000;

// Extension point: /api/survivors is a real endpoint but detection isn't
// implemented in onboard-autonomy yet, so the list is always empty
// today. Renders an explicit placeholder rather than sample rows.
export default function SurvivorsPanel() {
  const [survivors, setSurvivors] = useState<SurvivorResponse[]>([]);
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    let mounted = true;

    async function poll() {
      try {
        const s = await getSurvivors();
        if (mounted) {
          setSurvivors(s);
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

  return (
    <Panel title="Survivors">
      {error && <div className="text-bad text-xs mb-2">survivors fetch failed: {error}</div>}
      {survivors.length === 0 ? (
        <div className="text-dim text-xs">
          No survivors detected yet — detection not implemented in onboard-autonomy.
        </div>
      ) : (
        <div className="text-xs space-y-1">
          {survivors.map((s) => (
            <div key={s.survivor_id} className="flex justify-between border-b border-b-[#1d252d] last:border-b-0 py-0.5">
              <span className="text-dim">#{s.survivor_id}</span>
              <span className="tabular-nums">
                ({s.x.toFixed(2)}, {s.y.toFixed(2)}) conf {(s.confidence * 100).toFixed(0)}%
              </span>
            </div>
          ))}
        </div>
      )}
    </Panel>
  );
}
