import Panel, { Row } from "./Panel";
import { BoolPill } from "./Pill";
import { mavStateLabel } from "../format";
import type { TelemetryResponse } from "../types";

export default function FlightPanel({ telemetry }: { telemetry: TelemetryResponse | null }) {
  const fcu = telemetry?.fcu;
  return (
    <Panel title="Flight">
      <Row label="Armed">
        <BoolPill value={fcu?.armed ?? null} okLabel="ARMED" badLabel="DISARMED" />
      </Row>
      <Row label="Mode">{fcu?.mode ?? "—"}</Row>
      <Row label="System status">
        {fcu?.system_status != null ? `${fcu.system_status} (${mavStateLabel(fcu.system_status)})` : "—"}
      </Row>
      <Row label="Guided">
        <BoolPill value={fcu?.guided ?? null} />
      </Row>
    </Panel>
  );
}
