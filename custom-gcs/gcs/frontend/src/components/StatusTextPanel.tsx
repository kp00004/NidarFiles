import Panel from "./Panel";
import type { StatusTextResponse, TelemetryResponse } from "../types";

// MAVLink MAV_SEVERITY: 0..3 = problems, 4 = warning, 5..7 = information.
const SEVERITY: Record<number, { label: string; className: string }> = {
  0: { label: "EMERGENCY", className: "bg-bad text-white" },
  1: { label: "ALERT", className: "bg-bad text-white" },
  2: { label: "CRITICAL", className: "bg-bad text-white" },
  3: { label: "ERROR", className: "bg-bad text-white" },
  4: { label: "WARNING", className: "bg-warn text-black" },
  5: { label: "NOTICE", className: "bg-[#2b3a48] text-text" },
  6: { label: "INFO", className: "bg-[#2b3a48] text-dim" },
  7: { label: "DEBUG", className: "bg-[#2b3a48] text-dim" },
};

// Plain-language hints for ArduPilot messages seen on this drone. A hint
// explains; it never replaces the original text.
const HINTS: [RegExp, string][] = [
  [/Need Position Estimate/i, "No indoor position yet: check optical flow (light, texture) and the EKF origin."],
  [/Check mag field/i, "Compass reads a different magnetic field than expected: calibrate the compass, keep away from metal/motors."],
  [/mag anomaly/i, "Compass disturbed (metal or wiring nearby); heading was re-aligned."],
  [/Compass not healthy/i, "Selected compass not detected: check compass priority in Mission Planner."],
  [/Gyros inconsistent/i, "IMUs disagree: keep the drone still after power-on, let it warm up, or redo accel calibration."],
  [/Battery.*(failsafe|minimum|below)/i, "Battery too low for arming: charge it (or check BATT_* limits)."],
  [/Battery.*unhealthy/i, "Battery monitor not reading: check the power module connection."],
  [/stopped aiding/i, "EKF lost its position source (optical flow)."],
  [/is using optical flow|fusing optical flow/i, "EKF is using the optical flow sensor."],
  [/motor test/i, "Motor test step from the Motor Test mission."],
];

function hintFor(text: string): string | null {
  for (const [pattern, hint] of HINTS) if (pattern.test(text)) return hint;
  return null;
}

interface Row extends StatusTextResponse {
  count: number;
}

// Newest first; consecutive repeats of the same message become one row with a count.
export function collapse(messages: StatusTextResponse[]): Row[] {
  const rows: Row[] = [];
  for (const m of messages.slice().reverse()) {
    const last = rows[rows.length - 1];
    if (last && last.text === m.text && last.severity === m.severity) last.count += 1;
    else rows.push({ ...m, count: 1 });
  }
  return rows;
}

export default function StatusTextPanel({ telemetry }: { telemetry: TelemetryResponse | null }) {
  const rows = collapse(telemetry?.statustext ?? []);

  return (
    <Panel title="FCU messages (newest first)" className="col-span-full">
      <div className="max-h-56 overflow-y-auto">
        {rows.length === 0 ? (
          <div className="text-dim text-sm">no messages received</div>
        ) : (
          rows.map((m, i) => {
            const sev = SEVERITY[m.severity ?? 6] ?? SEVERITY[6];
            const hint = m.text ? hintFor(m.text) : null;
            return (
              <div key={i} className="flex gap-2.5 items-start py-1.5 border-b border-b-[#1d252d] last:border-b-0">
                <span className={`shrink-0 w-[78px] text-center text-[10px] font-bold rounded px-1 py-0.5 ${sev.className}`}>
                  {sev.label}
                </span>
                <div className="min-w-0">
                  <div className="text-sm text-text break-words">
                    {m.text}
                    {m.count > 1 && <span className="ml-2 text-dim text-xs">×{m.count}</span>}
                  </div>
                  {hint && <div className="text-xs text-dim mt-0.5">{hint}</div>}
                </div>
              </div>
            );
          })
        )}
      </div>
    </Panel>
  );
}
