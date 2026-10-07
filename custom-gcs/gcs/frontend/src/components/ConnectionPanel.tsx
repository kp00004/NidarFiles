import Panel, { Row } from "./Panel";
import { ConnPill } from "./Pill";
import { fmtNum } from "../format";
import type { TelemetryResponse } from "../types";

// Three missed 1 Hz heartbeats (matching the 1000ms poll cadence in
// useTelemetry) is treated as stale, same threshold whether the staleness
// comes from a null heartbeat_age_s (never received) or an old one.
const STALE_HEARTBEAT_S = 3;

export default function ConnectionPanel({ telemetry }: { telemetry: TelemetryResponse | null }) {
  const heartbeatAge = telemetry?.heartbeat_age_s ?? null;
  const isStale = heartbeatAge === null || heartbeatAge > STALE_HEARTBEAT_S;

  return (
    <Panel title="Connection">
      <Row label="GCS ↔ Jetson">
        <ConnPill value={telemetry?.connected ?? null} />
      </Row>
      <Row label="Jetson ↔ Pixhawk (FCU)">
        <ConnPill value={telemetry?.fcu.connected ?? null} />
      </Row>
      <Row label="Heartbeat age">
        <span className={isStale ? "text-warn" : undefined}>
          {heartbeatAge === null ? "no heartbeat received" : `${fmtNum(heartbeatAge, 1)} s`}
        </span>
      </Row>
    </Panel>
  );
}
