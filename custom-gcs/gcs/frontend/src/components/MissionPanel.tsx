import Panel, { Row } from "./Panel";
import type { TelemetryResponse } from "../types";

export default function MissionPanel({ telemetry }: { telemetry: TelemetryResponse | null }) {
  return (
    <Panel title="Mission">
      <Row label="State">{telemetry?.mission_state ?? "—"}</Row>
    </Panel>
  );
}
