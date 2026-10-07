import Panel, { Row } from "./Panel";
import { fmtNum } from "../format";
import type { TelemetryResponse } from "../types";

export default function AttitudePanel({ telemetry }: { telemetry: TelemetryResponse | null }) {
  const a = telemetry?.attitude ?? null;
  return (
    <Panel title="Attitude">
      <Row label="Orientation quaternion (x,y,z,w)">
        {a ? `${fmtNum(a.x, 3)}, ${fmtNum(a.y, 3)}, ${fmtNum(a.z, 3)}, ${fmtNum(a.w, 3)}` : "unavailable"}
      </Row>
    </Panel>
  );
}
