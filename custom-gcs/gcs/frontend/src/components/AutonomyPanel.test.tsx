import { describe, expect, it } from "vitest";
import { render, screen } from "@testing-library/react";
import AutonomyPanel from "./AutonomyPanel";
import type { TelemetryResponse } from "../types";

const BASE: TelemetryResponse = {
  connected: true,
  mission_state: "searching",
  fcu: { connected: true, armed: true, guided: true, mode: "GUIDED", system_status: 4 },
  battery: { voltage: null, current: null, percentage: null },
  pose: { position: null },
  velocity: null,
  attitude: null,
  gps: null,
  statustext: [],
  heartbeat_age_s: null,
  autonomy: { state: null, objective: null, target: null, next_action: null },
  sensors: { slam: null, lidar: null, rangefinder: null, camera: null },
  mapping: {
    available: false, resolution_m: null, width_cells: null, height_cells: null,
    origin_x: null, origin_y: null, coverage_cell_size_m: null, explored_pct: null,
  },
  navigation: { target: null, frontier_count: null, candidate_count: null, blacklisted_count: null, geofence_breached: null },
};

describe("AutonomyPanel", () => {
  it("shows 'unavailable' for every field before any telemetry arrives", () => {
    render(<AutonomyPanel telemetry={null} />);
    expect(screen.getAllByText("unavailable").length).toBeGreaterThan(0);
  });

  it("renders real autonomy state, objective, and target", () => {
    render(
      <AutonomyPanel
        telemetry={{
          ...BASE,
          autonomy: {
            state: "SEARCHING_FRONTIER",
            objective: "Explore unexplored region",
            target: [4.2, 7.8],
            next_action: "Navigate to frontier",
          },
        }}
      />,
    );
    expect(screen.getByText("SEARCHING_FRONTIER")).toBeInTheDocument();
    expect(screen.getByText("Explore unexplored region")).toBeInTheDocument();
    expect(screen.getByText("4.20, 7.80")).toBeInTheDocument();
    expect(screen.getByText("Navigate to frontier")).toBeInTheDocument();
  });

  it("flags a geofence breach explicitly", () => {
    render(
      <AutonomyPanel
        telemetry={{
          ...BASE,
          navigation: { ...BASE.navigation, geofence_breached: true },
        }}
      />,
    );
    expect(screen.getByText("BREACHED")).toBeInTheDocument();
  });
});
