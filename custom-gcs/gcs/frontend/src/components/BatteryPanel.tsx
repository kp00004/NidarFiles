import Panel, { Row } from "./Panel";
import { fmtOrUnavailable } from "../format";
import type { TelemetryResponse } from "../types";

export default function BatteryPanel({ telemetry }: { telemetry: TelemetryResponse | null }) {
  const battery = telemetry?.battery;
  const pct = battery?.percentage != null ? battery.percentage * 100 : null;
  return (
    <Panel title="Battery">
      <Row label="Voltage">{fmtOrUnavailable(battery?.voltage, " V")}</Row>
      <Row label="Current">{fmtOrUnavailable(battery?.current, " A")}</Row>
      <Row label="Percentage">{fmtOrUnavailable(pct, " %", 0)}</Row>
    </Panel>
  );
}
