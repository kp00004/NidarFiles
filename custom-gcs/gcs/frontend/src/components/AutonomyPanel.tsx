import Panel, { Row } from "./Panel";
import { fmtNum } from "../format";
import type { TelemetryResponse } from "../types";

// Displays onboard-autonomy's structured "Active Thinking" state --
// fixed-vocabulary operational telemetry (state/objective/target/next
// action), never free-form or LLM-generated reasoning text. See
// CHECKPOINT/docs/gcs_telemetry_contract.md and
// onboard-autonomy/nidar_autonomy/telemetry_contract.py, which are the
// source of the vocabulary rendered here -- this panel does not invent
// labels of its own.
export default function AutonomyPanel({ telemetry }: { telemetry: TelemetryResponse | null }) {
  const a = telemetry?.autonomy ?? null;
  const s = telemetry?.sensors ?? null;
  const n = telemetry?.navigation ?? null;
  const m = telemetry?.mapping ?? null;

  return (
    <Panel title="Autonomy">
      <Row label="State">{a?.state ?? "unavailable"}</Row>
      <Row label="Objective">{a?.objective ?? "unavailable"}</Row>
      <Row label="Target">{a?.target ? `${fmtNum(a.target[0])}, ${fmtNum(a.target[1])}` : "none"}</Row>
      <Row label="Next action">{a?.next_action ?? "unavailable"}</Row>
      <Row label="Frontiers / candidates">
        {n?.frontier_count != null ? `${n.frontier_count} / ${n.candidate_count ?? "?"}` : "unavailable"}
      </Row>
      <Row label="Blacklisted goals">{n?.blacklisted_count ?? "unavailable"}</Row>
      <Row label="Geofence">
        {n?.geofence_breached == null ? "unavailable" : n.geofence_breached ? "BREACHED" : "ok"}
      </Row>
      <Row label="Coverage">{m?.explored_pct != null ? `${m.explored_pct}% searched` : "unavailable"}</Row>
      <Row label="SLAM / LiDAR">{s ? `${s.slam ?? "unknown"} / ${s.lidar ?? "unknown"}` : "unavailable"}</Row>
    </Panel>
  );
}
