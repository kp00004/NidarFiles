import { useState } from "react";
import { useTelemetry } from "./hooks/useTelemetry";
import ConnectionPanel from "./components/ConnectionPanel";
import FlightPanel from "./components/FlightPanel";
import MissionPanel from "./components/MissionPanel";
import ControlsPanel from "./components/ControlsPanel";
import BatteryPanel from "./components/BatteryPanel";
import PositionVelocityPanel from "./components/PositionVelocityPanel";
import GpsPanel from "./components/GpsPanel";
import AttitudePanel from "./components/AttitudePanel";
import StatusTextPanel from "./components/StatusTextPanel";
import MapPanel from "./components/MapPanel";
import AutonomyPanel from "./components/AutonomyPanel";
import SurvivorsPanel from "./components/SurvivorsPanel";
import CameraPanel from "./components/CameraPanel";
import PerceptionPanel from "./components/PerceptionPanel";
import SimulationPanel from "./components/SimulationPanel";
import Footer from "./components/Footer";

// Display tabs. Switching tabs only changes what is shown -- it is not an
// operator command and sends nothing to the drone.
const TABS = [
  { id: "indoor", label: "Indoor mission" },
  { id: "gps", label: "GPS missions" },
] as const;
type TabId = (typeof TABS)[number]["id"];

export default function App() {
  const { data: telemetry, error, lastUpdatedAt } = useTelemetry();
  const [tab, setTab] = useState<TabId>("indoor");

  // The simulation panel is dev/bench-only and gated OFF by default --
  // per custom-gcs/CLAUDE.md Important Constraint #1, the operator
  // command surface is exactly Start and Abort; a competition-deployed
  // build must not expose any additional clickable control, even one as
  // thoroughly isolated from real flight as this one is (see
  // CHECKPOINT/docs/simulation_architecture.md for the isolation
  // guarantees). Opt in explicitly for local development/testing:
  //   VITE_ENABLE_SIMULATION=true npm run dev
  //   VITE_ENABLE_SIMULATION=true npm run build
  // Never set this for a competition build. See README.md. Read inside
  // the component (not as a module-level constant) so it reflects the
  // environment at render time, not just at first import.
  const simulationEnabled = import.meta.env.VITE_ENABLE_SIMULATION === "true";

  const subtitle = error
    ? `telemetry fetch failed: ${error}`
    : lastUpdatedAt
      ? `last updated ${lastUpdatedAt.toLocaleTimeString()}`
      : "connecting...";

  return (
    <div className="p-4">
      <h1 className="text-lg font-semibold mb-1">NIDAR AirMouse — Operator Panel</h1>
      <div className={`text-xs mb-4 ${error ? "text-bad font-semibold" : "text-dim"}`}>
        {subtitle}
      </div>

      <div role="tablist" aria-label="Panel view" className="flex gap-1 mb-3 border-b border-border">
        {TABS.map((t) => (
          <button
            key={t.id}
            type="button"
            role="tab"
            aria-selected={tab === t.id}
            className={`px-3 py-1.5 text-sm rounded-t-md border border-b-0 ${
              tab === t.id ? "bg-panel border-border text-text font-semibold" : "border-transparent text-dim hover:text-text"
            }`}
            onClick={() => setTab(t.id)}
          >
            {t.label}
          </button>
        ))}
      </div>

      {/* The Indoor panels stay mounted on the GPS tab (just hidden), so Mission
          Control keeps its state and polling -- START/ABORT are one click away. */}
      <div
        data-testid="indoor-panels"
        className={tab === "indoor" ? "grid grid-cols-[repeat(auto-fit,minmax(260px,1fr))] gap-3" : "hidden"}
      >
        <ConnectionPanel telemetry={telemetry} />
        <FlightPanel telemetry={telemetry} />
        <MissionPanel telemetry={telemetry} />
        <ControlsPanel telemetry={telemetry} />
        <BatteryPanel telemetry={telemetry} />
        <PositionVelocityPanel telemetry={telemetry} />
        <AttitudePanel telemetry={telemetry} />
        <AutonomyPanel telemetry={telemetry} />
        <SurvivorsPanel />
        <PerceptionPanel />
        <StatusTextPanel telemetry={telemetry} />
        <MapPanel telemetry={telemetry} />
        <CameraPanel />
      </div>

      {tab === "gps" && (
        <div className="grid grid-cols-[repeat(auto-fit,minmax(260px,1fr))] gap-3">
          <GpsPanel telemetry={telemetry} />
        </div>
      )}

      {simulationEnabled && (
        <>
          <div className="mt-6 mb-2 text-xs uppercase tracking-wide text-violet-400 font-semibold">
            Simulation — independent of the panels above, never touches real flight
          </div>
          <div className="grid grid-cols-[repeat(auto-fit,minmax(260px,1fr))] gap-3">
            <SimulationPanel />
          </div>
        </>
      )}

      <Footer />
    </div>
  );
}
