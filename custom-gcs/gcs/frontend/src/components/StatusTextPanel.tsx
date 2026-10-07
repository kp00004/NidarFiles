import Panel from "./Panel";
import type { TelemetryResponse } from "../types";

export default function StatusTextPanel({ telemetry }: { telemetry: TelemetryResponse | null }) {
  const messages = telemetry?.statustext ?? [];
  // Most recent first -- statustext arrives in receipt order, oldest first.
  const reversed = messages.slice().reverse();

  return (
    <Panel title="FCU status text / warnings" className="col-span-full">
      <div className="max-h-32 overflow-y-auto font-mono text-xs">
        {reversed.length === 0 ? (
          <div>no messages received</div>
        ) : (
          reversed.map((m, i) => (
            <div key={i} className="py-0.5 border-b border-b-[#1d252d] last:border-b-0">
              [sev {m.severity}] {m.text}
            </div>
          ))
        )}
      </div>
    </Panel>
  );
}
