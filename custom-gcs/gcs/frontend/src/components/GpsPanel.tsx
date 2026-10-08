import Panel, { Row } from "./Panel";
import { fmtNum, gpsFixLabel } from "../format";
import type { TelemetryResponse } from "../types";

// GPS details -- only meaningful for GPS (outdoor) missions. The indoor
// NIDAR AirMouse drone has no GPS fitted (GPS1_TYPE=0, GPS2_TYPE=0) and
// the radio telemetry does not relay GPS, so this stays "unavailable"
// for the indoor missions.
export default function GpsPanel({ telemetry }: { telemetry: TelemetryResponse | null }) {
  const gps = telemetry?.gps ?? null;

  return (
    <Panel title="GPS (GPS missions only)">
      <div className="mb-2 text-xs text-dim">
        For GPS-only (outdoor) missions. The indoor drone has no GPS, so these stay unavailable there.
      </div>
      <Row label="GPS fix">
        {gps?.fix_status != null ? `${gps.fix_status} (${gpsFixLabel(gps.fix_status)})` : "unavailable"}
      </Row>
      <Row label="Satellites">{gps?.satellites_visible ?? "unavailable"}</Row>
      <Row label="Latitude">{gps?.latitude != null ? `${fmtNum(gps.latitude, 7)}°` : "unavailable"}</Row>
      <Row label="Longitude">{gps?.longitude != null ? `${fmtNum(gps.longitude, 7)}°` : "unavailable"}</Row>
      <Row label="Altitude (MSL)">{gps?.altitude != null ? `${fmtNum(gps.altitude, 1)} m` : "unavailable"}</Row>
    </Panel>
  );
}
