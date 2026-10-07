import { useEffect, useState } from "react";
import Panel, { Row } from "./Panel";
import { getFlightTestStatus, getMissions, getRadioStatus, postAbort, postMissionStart } from "../api";
import type { FlightTestStatusResponse, Mission, RadioStatusResponse, TelemetryResponse } from "../types";

// The operator command surface is exactly START (for the selected mission)
// and ABORT -- see custom-gcs/CLAUDE.md Important Constraints #1. Choosing
// a mission in the dropdown is configuration, not a command. Do not add any
// other button that sends a command, and do not add a confirmation step
// before ABORT: abort must preempt everything immediately, per
// onboard-autonomy/CLAUDE.md Hard Safety Rule 2.
//
// Both commands go over the MicroLR900 command radio to the Jetson. What
// this panel shows is only what was actually confirmed:
//   1. sent over the radio / Jetson ACK (accepted or rejected with reason)
//   2. mission state from the Jetson (radio heartbeat, and the mission's status relayed over the radio)
//   3. vehicle state from telemetry (relayed over the radio)

const POLL_INTERVAL_MS = 1000;

type Outcome = { ok: boolean; text: string } | null;

function radioSummary(radio: RadioStatusResponse | null): { text: string; className: string } {
  if (!radio) return { text: "—", className: "text-dim" };
  if (!radio.enabled) return { text: "RADIO DISABLED", className: "text-bad font-semibold" };
  if (!radio.port_open) {
    return {
      text: `PORT ${radio.port ?? "?"} NOT OPEN${radio.port_error ? ` (${radio.port_error})` : ""}`,
      className: "text-bad font-semibold",
    };
  }
  if (!radio.jetson_link_up) return { text: "RADIO LINK DOWN — no Jetson heartbeat", className: "text-bad font-semibold" };
  return {
    text: `LINK UP (Jetson heartbeat ${radio.jetson_heartbeat_age_s ?? "?"} s ago)`,
    className: "text-ok font-semibold",
  };
}

export default function ControlsPanel({ telemetry }: { telemetry: TelemetryResponse | null }) {
  const [missions, setMissions] = useState<Mission[] | null>(null);
  const [missionsError, setMissionsError] = useState<string | null>(null);
  const [selectedId, setSelectedId] = useState("");
  const [radio, setRadio] = useState<RadioStatusResponse | null>(null);
  const [mission, setMission] = useState<FlightTestStatusResponse | null>(null);
  // Independent busy/outcome state per button, deliberately -- a slow START
  // (radio retries) must never disable ABORT. See docs/DECISIONS.md D-10.
  const [startBusy, setStartBusy] = useState(false);
  const [abortBusy, setAbortBusy] = useState(false);
  const [startOutcome, setStartOutcome] = useState<Outcome>(null);
  const [abortOutcome, setAbortOutcome] = useState<Outcome>(null);

  useEffect(() => {
    let mounted = true;
    getMissions()
      .then((m) => {
        if (!mounted) return;
        setMissions(m);
        setSelectedId((prev) => prev || m[0]?.id || "");
      })
      .catch((e) => mounted && setMissionsError(e instanceof Error ? e.message : String(e)));
    return () => {
      mounted = false;
    };
  }, []);

  useEffect(() => {
    let mounted = true;
    async function poll() {
      const [r, f] = await Promise.allSettled([getRadioStatus(), getFlightTestStatus()]);
      if (!mounted) return;
      setRadio(r.status === "fulfilled" ? r.value : null);
      setMission(f.status === "fulfilled" ? f.value : null);
    }
    poll();
    const id = setInterval(poll, POLL_INTERVAL_MS);
    return () => {
      mounted = false;
      clearInterval(id);
    };
  }, []);

  const selected = missions?.find((m) => m.id === selectedId) ?? null;

  async function start() {
    if (startBusy || !selected) return;
    setStartBusy(true);
    setStartOutcome({ ok: true, text: `Sending START (${selected.name}) over the radio…` });
    try {
      const res = await postMissionStart(selected.id);
      setStartOutcome({
        ok: true,
        text: `Jetson ACCEPTED START (${res.mission}) — ${res.attempts} radio attempt${res.attempts === 1 ? "" : "s"}`,
      });
    } catch (e) {
      setStartOutcome({ ok: false, text: `START not accepted: ${e instanceof Error ? e.message : String(e)}` });
    } finally {
      setStartBusy(false);
    }
  }

  async function abort() {
    if (abortBusy) return;
    setAbortBusy(true);
    setAbortOutcome({ ok: true, text: "Sending ABORT over the radio…" });
    try {
      const res = await postAbort();
      setAbortOutcome({ ok: true, text: `Jetson ACCEPTED ABORT — ${res.attempts} radio attempt${res.attempts === 1 ? "" : "s"}` });
    } catch (e) {
      setAbortOutcome({
        ok: false,
        text: `ABORT not confirmed: ${e instanceof Error ? e.message : String(e)} — USE THE RC TRANSMITTER / KILL SWITCH`,
      });
    } finally {
      setAbortBusy(false);
    }
  }

  const link = radioSummary(radio);
  const fcu = telemetry?.fcu;

  return (
    <Panel title="Mission Control">
      {missionsError && <div className="mb-2.5 text-bad text-xs font-semibold">{missionsError}</div>}

      <label className="block text-xs text-dim mb-3">
        Mission
        <select
          aria-label="Mission"
          className="mt-1 w-full bg-black/30 border border-border rounded px-2 py-1.5 text-sm text-text"
          value={selectedId}
          disabled={!missions || startBusy}
          onChange={(e) => setSelectedId(e.target.value)}
        >
          {(missions ?? []).map((m) => (
            <option key={m.id} value={m.id} className="bg-panel text-text">
              {m.name}
            </option>
          ))}
        </select>
      </label>

      <div className="flex gap-2.5 flex-wrap">
        <button
          type="button"
          className="font-semibold px-4 py-2.5 rounded-md border border-ok bg-ok text-white disabled:opacity-40 disabled:cursor-not-allowed"
          disabled={startBusy || !selected}
          onClick={start}
        >
          START
        </button>
        <button
          type="button"
          className="font-semibold px-4 py-2.5 rounded-md border border-bad bg-bad text-white disabled:opacity-40 disabled:cursor-not-allowed"
          disabled={abortBusy}
          onClick={abort}
        >
          STOP / ABORT
        </button>
      </div>

      {startOutcome && (
        <div className={`mt-2.5 text-xs font-semibold ${startOutcome.ok ? "text-ok" : "text-bad"}`}>{startOutcome.text}</div>
      )}
      {abortOutcome && (
        <div className={`mt-2.5 text-xs font-semibold ${abortOutcome.ok ? "text-ok" : "text-bad"}`}>{abortOutcome.text}</div>
      )}

      <div className="mt-3 text-xs">
        <Row label="Command radio">
          <span className={link.className}>{link.text}</span>
        </Row>
        <Row label="Mission (radio)">{radio?.jetson_link_up ? radio.jetson_mission_state ?? "—" : "—"}</Row>
        <Row label="Mission status">
          {mission?.state ? `${mission.state}${mission.execution_mode ? ` [${mission.execution_mode}]` : ""}` : "—"}
        </Row>
        {mission?.detail && <div className="mt-1 text-dim">{mission.detail}</div>}
        <Row label="Vehicle">
          {fcu?.connected
            ? `${fcu.armed ? "ARMED" : "disarmed"} · ${fcu.mode ?? "?"}${
                mission?.current_altitude_m != null ? ` · alt ${mission.current_altitude_m} m` : ""
              }`
            : "FCU not connected / no telemetry"}
        </Row>
      </div>

      <div className="mt-2.5 px-2.5 py-2 bg-warn/10 border border-warn/40 rounded-md text-warn text-xs font-semibold">
        REAL FLIGHT. START (Hover) makes the Jetson put the vehicle in GUIDED, arm, take off, hover and land.
        ABORT while airborne commands LAND. Keep the RC transmitter in hand as the independent override.
      </div>
    </Panel>
  );
}
