// Shared display-formatting helpers, mirroring the old prototype's
// fmtNum/fmtBool conventions (see git history: pre-React
// gcs/frontend/index.html).

export function fmtNum(n: number | null | undefined, digits = 2): string {
  return typeof n === "number" ? n.toFixed(digits) : "—";
}

export function fmtOrUnavailable(n: number | null | undefined, suffix = "", digits = 2): string {
  return typeof n === "number" ? `${n.toFixed(digits)}${suffix}` : "unavailable";
}

// MAVLink common MAV_STATE enum — see
// https://mavlink.io/en/messages/common.html#MAV_STATE. Bare-number
// system_status has been a real source of operator confusion (a `5`
// looks alarming until you know it's CRITICAL, not e.g. a percentage) —
// see CHECKPOINT/CURRENT_STATE.md §17.
const MAV_STATE_LABELS: Record<number, string> = {
  0: "UNINIT",
  1: "BOOT",
  2: "CALIBRATING",
  3: "STANDBY",
  4: "ACTIVE",
  5: "CRITICAL",
  6: "EMERGENCY",
  7: "POWEROFF",
  8: "FLIGHT_TERMINATION",
};

export function mavStateLabel(code: number | null | undefined): string {
  if (code == null) return "—";
  return MAV_STATE_LABELS[code] ?? `UNKNOWN(${code})`;
}

// MAVLink common GPS_FIX_TYPE enum — see
// https://mavlink.io/en/messages/common.html#GPS_FIX_TYPE.
const GPS_FIX_LABELS: Record<number, string> = {
  0: "NO_GPS",
  1: "NO_FIX",
  2: "FIX_2D",
  3: "FIX_3D",
  4: "DGPS",
  5: "RTK_FLOAT",
  6: "RTK_FIXED",
  7: "STATIC",
  8: "PPP",
};

export function gpsFixLabel(code: number | null | undefined): string {
  if (code == null) return "unavailable";
  return GPS_FIX_LABELS[code] ?? `UNKNOWN(${code})`;
}
