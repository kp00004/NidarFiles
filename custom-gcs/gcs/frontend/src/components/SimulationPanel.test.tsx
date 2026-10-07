import { afterEach, describe, expect, it, vi } from "vitest";
import { act, fireEvent, render, screen, waitFor } from "@testing-library/react";
import SimulationPanel from "./SimulationPanel";
import * as api from "../api";
import type { SimulationStatusResponse } from "../types";

const EMPTY_MAP = { resolution: null, width: null, height: null, data: null };
const EMPTY_PATH = { points: [] };

const IDLE_STATUS: SimulationStatusResponse = {
  source: "simulation",
  status: "idle",
  mission_state: "idle",
  step: 0,
  elapsed_sim_seconds: 0,
  pose: null,
  autonomy: { state: null, objective: null, target: null, next_action: null },
  sensors: { slam: null, lidar: null, rangefinder: null, camera: null },
  mapping: {
    available: false, resolution_m: null, width_cells: null, height_cells: null,
    origin_x: null, origin_y: null, coverage_cell_size_m: null, explored_pct: null,
  },
  navigation: { target: null, frontier_count: null, candidate_count: null, blacklisted_count: null, geofence_breached: null },
  map_known_pct: 0,
  coverage_search_pct: 0,
  error: null,
};

function mockSimulationApis(status: SimulationStatusResponse = IDLE_STATUS) {
  vi.spyOn(api, "getSimulationStatus").mockResolvedValue(status);
  vi.spyOn(api, "getSimulationMap").mockResolvedValue(EMPTY_MAP);
  vi.spyOn(api, "getSimulationCoverage").mockResolvedValue(EMPTY_MAP);
  vi.spyOn(api, "getSimulationPath").mockResolvedValue(EMPTY_PATH);
}

afterEach(() => {
  vi.restoreAllMocks();
});

describe("SimulationPanel", () => {
  it("shows the SIMULATION badge unconditionally", async () => {
    mockSimulationApis();
    render(<SimulationPanel />);
    expect(screen.getByText(/SIMULATION — NOT REAL FLIGHT/)).toBeInTheDocument();
    await waitFor(() => expect(api.getSimulationStatus).toHaveBeenCalled());
  });

  it("shows a RUN SIMULATION button, never labeled START", async () => {
    mockSimulationApis();
    render(<SimulationPanel />);
    expect(screen.getByRole("button", { name: "RUN SIMULATION" })).toBeInTheDocument();
    expect(screen.queryByRole("button", { name: "START" })).not.toBeInTheDocument();
    await waitFor(() => expect(api.getSimulationStatus).toHaveBeenCalled());
  });

  it("RUN SIMULATION calls postSimulationCommand('run'), never the real START/ABORT", async () => {
    mockSimulationApis();
    const postSimSpy = vi
      .spyOn(api, "postSimulationCommand")
      .mockResolvedValue({ status: "sent", command: "run" });
    const startSpy = vi.spyOn(api, "postMissionStart");
    const abortSpy = vi.spyOn(api, "postAbort");

    render(<SimulationPanel />);
    await act(async () => {
      fireEvent.click(screen.getByRole("button", { name: "RUN SIMULATION" }));
    });

    expect(postSimSpy).toHaveBeenCalledWith("run");
    expect(startSpy).not.toHaveBeenCalled();
    expect(abortSpy).not.toHaveBeenCalled();
  });

  it("RESET calls postSimulationCommand('reset')", async () => {
    mockSimulationApis();
    const postSimSpy = vi
      .spyOn(api, "postSimulationCommand")
      .mockResolvedValue({ status: "sent", command: "reset" });

    render(<SimulationPanel />);
    await act(async () => {
      fireEvent.click(screen.getByRole("button", { name: "RESET" }));
    });

    expect(postSimSpy).toHaveBeenCalledWith("reset");
  });

  it("shows a no-map placeholder before any run, not a fabricated grid", async () => {
    mockSimulationApis();
    render(<SimulationPanel />);
    await waitFor(() => expect(screen.getByText(/No simulation running yet/)).toBeInTheDocument());
    expect(screen.queryByTestId("simulation-map-canvas")).not.toBeInTheDocument();
  });

  it("renders live simulation status once data arrives", async () => {
    mockSimulationApis({
      ...IDLE_STATUS,
      status: "running",
      mission_state: "searching",
      step: 42,
      map_known_pct: 37.5,
      autonomy: { state: "SEARCHING_FRONTIER", objective: "Explore unexplored region", target: [4.2, 7.8], next_action: "Navigate to frontier" },
    });

    render(<SimulationPanel />);

    await waitFor(() => expect(screen.getByText("running")).toBeInTheDocument());
    expect(screen.getByText("searching")).toBeInTheDocument();
    expect(screen.getByText("37.5%")).toBeInTheDocument();
    expect(screen.getByText("SEARCHING_FRONTIER")).toBeInTheDocument();
  });

  it("renders the map canvas once real simulation map data arrives", async () => {
    vi.spyOn(api, "getSimulationStatus").mockResolvedValue({ ...IDLE_STATUS, status: "running" });
    vi.spyOn(api, "getSimulationMap").mockResolvedValue({
      resolution: 1, width: 5, height: 5, data: new Array(25).fill(0),
    });
    vi.spyOn(api, "getSimulationCoverage").mockResolvedValue(EMPTY_MAP);
    vi.spyOn(api, "getSimulationPath").mockResolvedValue(EMPTY_PATH);

    render(<SimulationPanel />);

    expect(await screen.findByTestId("simulation-map-canvas")).toBeInTheDocument();
    expect(screen.queryByText(/No simulation running yet/)).not.toBeInTheDocument();
  });

  it("shows a simulation error surfaced from the backend without crashing", async () => {
    mockSimulationApis({ ...IDLE_STATUS, status: "failed", error: "exceeded max_steps safety bound" });
    render(<SimulationPanel />);
    await waitFor(() =>
      expect(screen.getByText(/simulation error: exceeded max_steps safety bound/)).toBeInTheDocument(),
    );
  });
});
