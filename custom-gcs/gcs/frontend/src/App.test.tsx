import { afterEach, describe, expect, it, vi } from "vitest";
import { render, screen, waitFor } from "@testing-library/react";
import App from "./App";
import * as api from "./api";
import type { TelemetryResponse } from "./types";

const TELEMETRY: TelemetryResponse = {
  connected: true,
  mission_state: "searching",
  fcu: { connected: true, armed: true, guided: true, mode: "GUIDED", system_status: 4 },
  battery: { voltage: 12.4, current: 3.1, percentage: 0.75 },
  pose: { position: { x: 1, y: 2, z: 0.5 } },
  velocity: { x: 0.1, y: 0, z: 0 },
  attitude: { x: 0, y: 0, z: 0, w: 1 },
  gps: { fix_status: 3, satellites_visible: 9, latitude: 12.9, longitude: 77.6, altitude: 900 },
  statustext: [{ severity: 6, text: "boot complete" }],
  heartbeat_age_s: 0.3,
  autonomy: { state: "SEARCHING_FRONTIER", objective: "Explore unexplored region", target: null, next_action: "Navigate to frontier" },
  sensors: { slam: "ok", lidar: "ok", rangefinder: "not_integrated", camera: "not_integrated" },
  mapping: {
    available: false, resolution_m: null, width_cells: null, height_cells: null,
    origin_x: null, origin_y: null, coverage_cell_size_m: null, explored_pct: null,
  },
  navigation: { target: null, frontier_count: null, candidate_count: null, blacklisted_count: null, geofence_breached: null },
};

afterEach(() => {
  vi.restoreAllMocks();
});

describe("App", () => {
  it("shows the bench Setup section only when the backend enables it", async () => {
    vi.spyOn(api, "getTelemetry").mockResolvedValue(TELEMETRY);
    vi.spyOn(api, "getMissions").mockResolvedValue([]);
    const health = vi.spyOn(api, "getHealth").mockResolvedValue({
      connected: true, ros_status: "disabled", telemetry_source: "radio", setup_enabled: false,
      rosbridge_host: "127.0.0.1", rosbridge_port: 9090,
    });
    const { unmount } = render(<App />);
    await waitFor(() => expect(health).toHaveBeenCalled());
    expect(screen.queryByText("Pixhawk parameters")).not.toBeInTheDocument();
    expect(screen.queryByRole("tab")).not.toBeInTheDocument();
    unmount();
    health.mockResolvedValue({
      connected: true, ros_status: "disabled", telemetry_source: "radio", setup_enabled: true,
      rosbridge_host: "127.0.0.1", rosbridge_port: 9090,
    });
    render(<App />);
    const setupTab = await screen.findByRole("tab", { name: "Setup: Pixhawk parameters" });
    expect(screen.queryByText("Pixhawk parameters")).not.toBeInTheDocument();
    setupTab.click();
    expect(await screen.findByText("Pixhawk parameters")).toBeInTheDocument();
    expect(screen.getByTestId("mission-panels")).toHaveClass("hidden");
    screen.getByRole("tab", { name: "Mission" }).click();
    await waitFor(() => expect(screen.getByTestId("mission-panels")).not.toHaveClass("hidden"));
  });

  it("shows a separate GPS panel (GPS missions only) alongside the others", async () => {
    vi.spyOn(api, "getTelemetry").mockResolvedValue(TELEMETRY);
    vi.spyOn(api, "getMissions").mockResolvedValue([]);
    render(<App />);
    expect(await screen.findByText(/GPS \(GPS missions only\)/)).toBeInTheDocument();
    expect(screen.getByText("Satellites")).toBeInTheDocument();
    expect(screen.queryByRole("tab")).not.toBeInTheDocument();
  });

  it("renders operator panels and reflects telemetry once loaded", async () => {
    vi.spyOn(api, "getTelemetry").mockResolvedValue(TELEMETRY);
    vi.spyOn(api, "getMap").mockResolvedValue({ resolution: null, width: null, height: null, data: null });
    vi.spyOn(api, "getCoverage").mockResolvedValue({ resolution: null, width: null, height: null, data: null });
    vi.spyOn(api, "getPath").mockResolvedValue({ points: [] });
    vi.spyOn(api, "getSimulationStatus").mockResolvedValue({
      source: "simulation", status: "idle", mission_state: "idle", step: 0, elapsed_sim_seconds: 0,
      pose: null,
      autonomy: { state: null, objective: null, target: null, next_action: null },
      sensors: { slam: null, lidar: null, rangefinder: null, camera: null },
      mapping: { available: false, resolution_m: null, width_cells: null, height_cells: null, origin_x: null, origin_y: null, coverage_cell_size_m: null, explored_pct: null },
      navigation: { target: null, frontier_count: null, candidate_count: null, blacklisted_count: null, geofence_breached: null },
      map_known_pct: 0, coverage_search_pct: 0, error: null,
    });
    vi.spyOn(api, "getSimulationMap").mockResolvedValue({ resolution: null, width: null, height: null, data: null });
    vi.spyOn(api, "getSimulationCoverage").mockResolvedValue({ resolution: null, width: null, height: null, data: null });
    vi.spyOn(api, "getSimulationPath").mockResolvedValue({ points: [] });
    vi.spyOn(api, "getSurvivors").mockResolvedValue([]);
    vi.spyOn(api, "getPerceptionDetections").mockResolvedValue({
      frame_width: null, frame_height: null, timestamp: null, detections: [],
    });
    vi.spyOn(api, "getPerceptionStatus").mockResolvedValue({
      camera_connected: null, detector_enabled: null, detector_ready: null, detector_backend: null,
      model_name: null, person_count: null, fps: null, frame_width: null, frame_height: null,
      last_detection_age_s: null,
    });
    vi.spyOn(api, "getCameraStatus").mockResolvedValue({
      connected: null, stream_url: null, frame_width: null, frame_height: null, fps: null,
    });
    vi.spyOn(api, "getHealth").mockResolvedValue({
      connected: true,
      ros_status: "connected",
      rosbridge_host: "127.0.0.1",
      rosbridge_port: 9090,
    });

    render(<App />);

    expect(screen.getByText("NIDAR AirMouse — Operator Panel")).toBeInTheDocument();
    await waitFor(() => expect(screen.getByText("searching")).toBeInTheDocument());
    expect(screen.getByRole("button", { name: "START" })).toBeInTheDocument();
    expect(screen.getByRole("button", { name: "STOP / ABORT" })).toBeInTheDocument();
    await waitFor(() =>
      expect(screen.getByText(/rosbridge target: 127\.0\.0\.1:9090/)).toBeInTheDocument(),
    );
  });

  it("shows a telemetry-fetch-failed banner instead of a blank page on failure", async () => {
    vi.spyOn(api, "getTelemetry").mockRejectedValue(new Error("network error"));
    vi.spyOn(api, "getMap").mockResolvedValue({ resolution: null, width: null, height: null, data: null });
    vi.spyOn(api, "getCoverage").mockResolvedValue({ resolution: null, width: null, height: null, data: null });
    vi.spyOn(api, "getPath").mockResolvedValue({ points: [] });
    vi.spyOn(api, "getSimulationStatus").mockResolvedValue({
      source: "simulation", status: "idle", mission_state: "idle", step: 0, elapsed_sim_seconds: 0,
      pose: null,
      autonomy: { state: null, objective: null, target: null, next_action: null },
      sensors: { slam: null, lidar: null, rangefinder: null, camera: null },
      mapping: { available: false, resolution_m: null, width_cells: null, height_cells: null, origin_x: null, origin_y: null, coverage_cell_size_m: null, explored_pct: null },
      navigation: { target: null, frontier_count: null, candidate_count: null, blacklisted_count: null, geofence_breached: null },
      map_known_pct: 0, coverage_search_pct: 0, error: null,
    });
    vi.spyOn(api, "getSimulationMap").mockResolvedValue({ resolution: null, width: null, height: null, data: null });
    vi.spyOn(api, "getSimulationCoverage").mockResolvedValue({ resolution: null, width: null, height: null, data: null });
    vi.spyOn(api, "getSimulationPath").mockResolvedValue({ points: [] });
    vi.spyOn(api, "getSurvivors").mockResolvedValue([]);
    vi.spyOn(api, "getPerceptionDetections").mockResolvedValue({
      frame_width: null, frame_height: null, timestamp: null, detections: [],
    });
    vi.spyOn(api, "getPerceptionStatus").mockResolvedValue({
      camera_connected: null, detector_enabled: null, detector_ready: null, detector_backend: null,
      model_name: null, person_count: null, fps: null, frame_width: null, frame_height: null,
      last_detection_age_s: null,
    });
    vi.spyOn(api, "getCameraStatus").mockResolvedValue({
      connected: null, stream_url: null, frame_width: null, frame_height: null, fps: null,
    });
    vi.spyOn(api, "getHealth").mockResolvedValue({
      connected: false,
      ros_status: "unavailable",
      rosbridge_host: "127.0.0.1",
      rosbridge_port: 9090,
    });

    render(<App />);

    expect(
      await screen.findByText(/telemetry fetch failed: network error/),
    ).toBeInTheDocument();
  });

  it("does NOT render the simulation panel by default (competition-build safety)", async () => {
    vi.spyOn(api, "getTelemetry").mockResolvedValue(TELEMETRY);
    vi.spyOn(api, "getMap").mockResolvedValue({ resolution: null, width: null, height: null, data: null });
    vi.spyOn(api, "getCoverage").mockResolvedValue({ resolution: null, width: null, height: null, data: null });
    vi.spyOn(api, "getPath").mockResolvedValue({ points: [] });
    vi.spyOn(api, "getSurvivors").mockResolvedValue([]);
    vi.spyOn(api, "getPerceptionDetections").mockResolvedValue({
      frame_width: null, frame_height: null, timestamp: null, detections: [],
    });
    vi.spyOn(api, "getPerceptionStatus").mockResolvedValue({
      camera_connected: null, detector_enabled: null, detector_ready: null, detector_backend: null,
      model_name: null, person_count: null, fps: null, frame_width: null, frame_height: null,
      last_detection_age_s: null,
    });
    vi.spyOn(api, "getCameraStatus").mockResolvedValue({
      connected: null, stream_url: null, frame_width: null, frame_height: null, fps: null,
    });
    vi.spyOn(api, "getHealth").mockResolvedValue({ connected: true, ros_status: "connected", rosbridge_host: "127.0.0.1", rosbridge_port: 9090 });
    const simStatusSpy = vi.spyOn(api, "getSimulationStatus");

    render(<App />);
    await waitFor(() => expect(screen.getByText("searching")).toBeInTheDocument());

    // No VITE_ENABLE_SIMULATION set in this test -- per
    // custom-gcs/CLAUDE.md Important Constraint #1, the simulation
    // controls must not exist in the operator-facing surface unless
    // explicitly opted into.
    expect(screen.queryByRole("button", { name: "RUN SIMULATION" })).not.toBeInTheDocument();
    expect(screen.queryByText(/SIMULATION — NOT REAL FLIGHT/)).not.toBeInTheDocument();
    expect(simStatusSpy).not.toHaveBeenCalled();
  });

  it("renders the simulation panel only when VITE_ENABLE_SIMULATION=true", async () => {
    vi.stubEnv("VITE_ENABLE_SIMULATION", "true");
    vi.spyOn(api, "getTelemetry").mockResolvedValue(TELEMETRY);
    vi.spyOn(api, "getMap").mockResolvedValue({ resolution: null, width: null, height: null, data: null });
    vi.spyOn(api, "getCoverage").mockResolvedValue({ resolution: null, width: null, height: null, data: null });
    vi.spyOn(api, "getPath").mockResolvedValue({ points: [] });
    vi.spyOn(api, "getSurvivors").mockResolvedValue([]);
    vi.spyOn(api, "getPerceptionDetections").mockResolvedValue({
      frame_width: null, frame_height: null, timestamp: null, detections: [],
    });
    vi.spyOn(api, "getPerceptionStatus").mockResolvedValue({
      camera_connected: null, detector_enabled: null, detector_ready: null, detector_backend: null,
      model_name: null, person_count: null, fps: null, frame_width: null, frame_height: null,
      last_detection_age_s: null,
    });
    vi.spyOn(api, "getCameraStatus").mockResolvedValue({
      connected: null, stream_url: null, frame_width: null, frame_height: null, fps: null,
    });
    vi.spyOn(api, "getHealth").mockResolvedValue({ connected: true, ros_status: "connected", rosbridge_host: "127.0.0.1", rosbridge_port: 9090 });
    vi.spyOn(api, "getSimulationStatus").mockResolvedValue({
      source: "simulation", status: "idle", mission_state: "idle", step: 0, elapsed_sim_seconds: 0,
      pose: null,
      autonomy: { state: null, objective: null, target: null, next_action: null },
      sensors: { slam: null, lidar: null, rangefinder: null, camera: null },
      mapping: { available: false, resolution_m: null, width_cells: null, height_cells: null, origin_x: null, origin_y: null, coverage_cell_size_m: null, explored_pct: null },
      navigation: { target: null, frontier_count: null, candidate_count: null, blacklisted_count: null, geofence_breached: null },
      map_known_pct: 0, coverage_search_pct: 0, error: null,
    });
    vi.spyOn(api, "getSimulationMap").mockResolvedValue({ resolution: null, width: null, height: null, data: null });
    vi.spyOn(api, "getSimulationCoverage").mockResolvedValue({ resolution: null, width: null, height: null, data: null });
    vi.spyOn(api, "getSimulationPath").mockResolvedValue({ points: [] });

    render(<App />);

    expect(await screen.findByRole("button", { name: "RUN SIMULATION" })).toBeInTheDocument();
    expect(screen.getByText(/SIMULATION — NOT REAL FLIGHT/)).toBeInTheDocument();

    vi.unstubAllEnvs();
  });

  it("renders Mission Control (Mission dropdown + START/ABORT) by default, no flag needed", async () => {
    vi.spyOn(api, "getTelemetry").mockResolvedValue(TELEMETRY);
    vi.spyOn(api, "getMap").mockResolvedValue({ resolution: null, width: null, height: null, data: null });
    vi.spyOn(api, "getCoverage").mockResolvedValue({ resolution: null, width: null, height: null, data: null });
    vi.spyOn(api, "getPath").mockResolvedValue({ points: [] });
    vi.spyOn(api, "getSurvivors").mockResolvedValue([]);
    vi.spyOn(api, "getPerceptionDetections").mockResolvedValue({
      frame_width: null, frame_height: null, timestamp: null, detections: [],
    });
    vi.spyOn(api, "getPerceptionStatus").mockResolvedValue({
      camera_connected: null, detector_enabled: null, detector_ready: null, detector_backend: null,
      model_name: null, person_count: null, fps: null, frame_width: null, frame_height: null,
      last_detection_age_s: null,
    });
    vi.spyOn(api, "getCameraStatus").mockResolvedValue({
      connected: null, stream_url: null, frame_width: null, frame_height: null, fps: null,
    });
    vi.spyOn(api, "getHealth").mockResolvedValue({ connected: true, ros_status: "connected", rosbridge_host: "127.0.0.1", rosbridge_port: 9090 });
    vi.spyOn(api, "getMissions").mockResolvedValue([{ id: "hover", name: "Hover", description: "" }]);

    render(<App />);
    await waitFor(() => expect(screen.getByText("searching")).toBeInTheDocument());

    expect(screen.getByText("Mission Control")).toBeInTheDocument();
    expect(await screen.findByRole("option", { name: "Hover" })).toBeInTheDocument();
    // The old dev-only alternate start panel is gone: one START, one ABORT.
    expect(screen.queryByText(/ALTERNATE START PATH/)).not.toBeInTheDocument();
    expect(screen.getAllByRole("button", { name: "START" })).toHaveLength(1);
  });
});
