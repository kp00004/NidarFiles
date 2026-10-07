import Panel, { Row } from "./Panel";
import { fmtNum, gpsFixLabel } from "../format";
import type { TelemetryResponse } from "../types";

export default function PositionVelocityPanel({ telemetry }: { telemetry: TelemetryResponse | null }) {
  const p = telemetry?.pose?.position ?? null;
  const v = telemetry?.velocity ?? null;
  const gps = telemetry?.gps ?? null;

  return (
    <Panel title="Position / Velocity">
      <Row label="Local position (x,y,z)">
        {p ? `${fmtNum(p.x)}, ${fmtNum(p.y)}, ${fmtNum(p.z)}` : "unavailable"}
      </Row>
      <Row label="Velocity (x,y,z)">
        {v ? `${fmtNum(v.x)}, ${fmtNum(v.y)}, ${fmtNum(v.z)}` : "unavailable"}
      </Row>
      <Row label="GPS fix">
        {gps?.fix_status != null ? `${gps.fix_status} (${gpsFixLabel(gps.fix_status)})` : "unavailable"}
      </Row>
      <Row label="Satellites">{gps?.satellites_visible ?? "unavailable"}</Row>
    </Panel>
  );
}
