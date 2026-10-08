import { useEffect, useState } from "react";
import { getHealth } from "./api";
import { useTelemetry } from "./hooks/useTelemetry";
import SetupPanel from "./components/SetupPanel";
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

export default function App() {
  const { data: telemetry, error, lastUpdatedAt } = useTelemetry();

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

  // Bench Setup section: shown only when the backend itself was started for
  // setup (start_gcs.ps1 -Setup) -- its routes don't exist otherwise.
  const [setupEnabled, setSetupEnabled] = useState(false);
  useEffect(() => {
    let mounted = true;
    getHealth()
      .then((h) => mounted && setSetupEnabled(Boolean(h.setup_enabled)))
      .catch(() => undefined);
    return () => {
      mounted = false;
    };
  }, []);

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

      <div className="grid grid-cols-[repeat(auto-fit,minmax(260px,1fr))] gap-3">
        <ConnectionPanel telemetry={telemetry} />
        <FlightPanel telemetry={telemetry} />
        <MissionPanel telemetry={telemetry} />
        <ControlsPanel telemetry={telemetry} />
        <BatteryPanel telemetry={telemetry} />
        <PositionVelocityPanel telemetry={telemetry} />
        <GpsPanel telemetry={telemetry} />
        <AttitudePanel telemetry={telemetry} />
        <AutonomyPanel telemetry={telemetry} />
        <SurvivorsPanel />
        <PerceptionPanel />
        <StatusTextPanel telemetry={telemetry} />
        <MapPanel telemetry={telemetry} />
        <CameraPanel />
      </div>


      {setupEnabled && (
        <>
          <div className="mt-6 mb-2 text-xs uppercase tracking-wide text-warn font-semibold">
            Bench setup — Pixhawk parameters (not for missions; writes refused while armed)
          </div>
          <div className="grid grid-cols-[repeat(auto-fit,minmax(260px,1fr))] gap-3">
            <SetupPanel />
          </div>
        </>
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
