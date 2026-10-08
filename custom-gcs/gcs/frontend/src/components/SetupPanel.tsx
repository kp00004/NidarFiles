import { useState } from "react";
import Panel from "./Panel";
import { getParam, setParam } from "../api";

// Bench setup only -- rendered only when the backend runs with
// GCS_SETUP_ENABLED (start_gcs.ps1 -Setup); a mission build never shows it.
// Reads/writes Pixhawk parameters over the radio through the Jetson, which
// refuses writes unless started with --setup, while armed, or while a
// mission runs. What is shown is always the value read back from the FCU.

// Parameters from the pre-flight checklist (README section 11), for one-click reading.
const QUICK = [
  "FS_GCS_ENABLE",
  "SYSID_MYGCS",
  "FS_EKF_ACTION",
  "BATT_FS_LOW_ACT",
  "RNGFND1_MAX_CM",
  "RNGFND1_MIN_CM",
  "RNGFND1_GNDCLEAR",
  "FLOW_ORIENT_YAW",
  "FLOW_FXSCALER",
  "FLOW_FYSCALER",
  "EK3_SRC_OPTIONS",
  "COMPASS_USE",
  "ARMING_CHECK",
  "BATT_ARM_VOLT",
  "BATT_LOW_VOLT",
  "BATT_CRT_VOLT",
  "GPS2_TYPE",
  "LOG_DISARMED",
];

type Line = { ok: boolean; text: string };

export default function SetupPanel() {
  const [name, setName] = useState("");
  const [value, setValue] = useState("");
  const [busy, setBusy] = useState(false);
  const [log, setLog] = useState<Line[]>([]);

  const add = (line: Line) => setLog((prev) => [line, ...prev].slice(0, 12));
  const cleanName = name.trim().toUpperCase();

  async function read(param = cleanName) {
    if (!param || busy) return;
    setName(param);
    setBusy(true);
    try {
      const r = await getParam(param);
      setValue(String(r.value));
      add({ ok: true, text: `${r.name} = ${r.value}` });
    } catch (e) {
      add({ ok: false, text: `${param}: ${e instanceof Error ? e.message : String(e)}` });
    } finally {
      setBusy(false);
    }
  }

  async function write() {
    const v = Number(value);
    if (!cleanName || value.trim() === "" || !Number.isFinite(v) || busy) return;
    setBusy(true);
    try {
      const r = await setParam(cleanName, v);
      add({ ok: true, text: `WROTE ${r.name} = ${r.value} (read back from the FCU)` });
    } catch (e) {
      add({ ok: false, text: `${cleanName}: ${e instanceof Error ? e.message : String(e)}` });
    } finally {
      setBusy(false);
    }
  }

  return (
    <Panel title="Pixhawk parameters" className="col-span-full">
      <div className="flex flex-wrap gap-2 items-end">
        <label className="text-xs text-dim">
          Name
          <input
            aria-label="Parameter name"
            className="block mt-1 w-56 bg-black/30 border border-border rounded px-2 py-1.5 text-sm text-text uppercase"
            value={name}
            onChange={(e) => setName(e.target.value)}
            onKeyDown={(e) => e.key === "Enter" && read()}
            placeholder="e.g. RNGFND1_MAX_CM"
          />
        </label>
        <button
          type="button"
          className="px-3 py-1.5 rounded-md border border-border text-sm disabled:opacity-40"
          disabled={busy || !cleanName}
          onClick={() => read()}
        >
          Read
        </button>
        <label className="text-xs text-dim">
          Value
          <input
            aria-label="Parameter value"
            className="block mt-1 w-32 bg-black/30 border border-border rounded px-2 py-1.5 text-sm text-text"
            value={value}
            onChange={(e) => setValue(e.target.value)}
            inputMode="decimal"
          />
        </label>
        <button
          type="button"
          className="px-3 py-1.5 rounded-md border border-warn text-warn text-sm font-semibold disabled:opacity-40"
          disabled={busy || !cleanName || value.trim() === "" || !Number.isFinite(Number(value))}
          onClick={write}
        >
          Write
        </button>
        {busy && <span className="text-xs text-dim">waiting for the Jetson…</span>}
      </div>

      <div className="mt-3 flex flex-wrap gap-1.5">
        {QUICK.map((p) => (
          <button
            key={p}
            type="button"
            className="px-2 py-0.5 rounded border border-border text-[11px] text-dim hover:text-text disabled:opacity-40"
            disabled={busy}
            onClick={() => read(p)}
          >
            {p}
          </button>
        ))}
      </div>

      <div className="mt-3 text-sm">
        {log.length === 0 ? (
          <div className="text-dim text-xs">
            Read a parameter by name or pick one above. Writes need the Jetson started with
            start_jetson.sh --setup and a disarmed vehicle.
          </div>
        ) : (
          log.map((l, i) => (
            <div key={i} className={`py-0.5 ${l.ok ? "text-text" : "text-bad"}`}>
              {l.text}
            </div>
          ))
        )}
      </div>
    </Panel>
  );
}
