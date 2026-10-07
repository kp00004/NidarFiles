import { describe, expect, it } from "vitest";
import { render, screen } from "@testing-library/react";
import ConnectionPanel from "./ConnectionPanel";
import type { TelemetryResponse } from "../types";

// This panel has its own state x event surface never previously covered
// by any test (App.test.tsx only ever renders it with `telemetry: null`
// or one fully-connected, fresh-heartbeat payload): the GCS<->Jetson link
// and the Jetson<->Pixhawk (FCU) link fail independently, and heartbeat
// staleness is a boundary condition (`> STALE_HEARTBEAT_S`, not `>=`).

function telemetryWith(overrides: Partial<TelemetryResponse>): TelemetryResponse {
  return {
    connected: true,
    mission_state: "idle",
    fcu: { connected: true, armed: false, guided: false, mode: "GUIDED", system_status: 3 },
    battery: { voltage: null, current: null, percentage: null },
    pose: { position: null },
    velocity: null,
    attitude: null,
    gps: null,
    statustext: [],
    heartbeat_age_s: 0.2,
    autonomy: { state: null, objective: null, target: null, next_action: null },
    sensors: { slam: null, lidar: null, rangefinder: null, camera: null },
    mapping: {
      available: false, resolution_m: null, width_cells: null, height_cells: null,
      origin_x: null, origin_y: null, coverage_cell_size_m: null, explored_pct: null,
    },
    navigation: { target: null, frontier_count: null, candidate_count: null, blacklisted_count: null, geofence_breached: null },
    ...overrides,
  };
}

describe("ConnectionPanel", () => {
  it("shows both links as UNKNOWN and 'no heartbeat received' before any telemetry arrives", () => {
    render(<ConnectionPanel telemetry={null} />);

    expect(screen.getAllByText("UNKNOWN")).toHaveLength(2);
    expect(screen.getByText("no heartbeat received")).toBeInTheDocument();
  });

  it("shows GCS<->Jetson CONNECTED independently of a disconnected FCU link", () => {
    render(
      <ConnectionPanel
        telemetry={telemetryWith({
          connected: true,
          fcu: { connected: false, armed: null, guided: null, mode: null, system_status: null },
        })}
      />,
    );

    expect(screen.getByText("CONNECTED")).toBeInTheDocument();
    expect(screen.getByText("DISCONNECTED")).toBeInTheDocument();
  });

  it("shows GCS<->Jetson DISCONNECTED independently of a connected FCU link", () => {
    render(
      <ConnectionPanel
        telemetry={telemetryWith({
          connected: false,
          fcu: { connected: true, armed: false, guided: false, mode: "GUIDED", system_status: 3 },
        })}
      />,
    );

    expect(screen.getByText("DISCONNECTED")).toBeInTheDocument();
    expect(screen.getByText("CONNECTED")).toBeInTheDocument();
  });

  it("does not flag the heartbeat as stale at exactly the 3s threshold", () => {
    render(<ConnectionPanel telemetry={telemetryWith({ heartbeat_age_s: 3 })} />);

    const value = screen.getByText("3.0 s");
    expect(value.className).not.toMatch(/text-warn/);
  });

  it("flags the heartbeat as stale just above the 3s threshold", () => {
    render(<ConnectionPanel telemetry={telemetryWith({ heartbeat_age_s: 3.1 })} />);

    const value = screen.getByText("3.1 s");
    expect(value.className).toMatch(/text-warn/);
  });

  it("treats a null heartbeat_age_s as stale even though connected is true", () => {
    render(<ConnectionPanel telemetry={telemetryWith({ connected: true, heartbeat_age_s: null })} />);

    const value = screen.getByText("no heartbeat received");
    expect(value.className).toMatch(/text-warn/);
  });

  it("does not flag a fresh heartbeat as stale", () => {
    render(<ConnectionPanel telemetry={telemetryWith({ heartbeat_age_s: 0.4 })} />);

    const value = screen.getByText("0.4 s");
    expect(value.className).not.toMatch(/text-warn/);
  });
});
