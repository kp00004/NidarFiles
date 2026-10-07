import { afterEach, describe, expect, it, vi } from "vitest";
import { render, screen, waitFor } from "@testing-library/react";
import MapPanel from "./MapPanel";
import * as api from "../api";

const EMPTY_COVERAGE = { resolution: null, width: null, height: null, data: null };
const EMPTY_PATH = { points: [] };
const EMPTY_FRONTIERS = { points: [] };

function mockMapApis(map: Awaited<ReturnType<typeof api.getMap>>) {
  vi.spyOn(api, "getMap").mockResolvedValue(map);
  vi.spyOn(api, "getCoverage").mockResolvedValue(EMPTY_COVERAGE);
  vi.spyOn(api, "getPath").mockResolvedValue(EMPTY_PATH);
  vi.spyOn(api, "getFrontiers").mockResolvedValue(EMPTY_FRONTIERS);
}

afterEach(() => {
  vi.restoreAllMocks();
});

describe("MapPanel", () => {
  it("shows an explicit no-map-yet placeholder when data is null, not a fabricated grid", async () => {
    mockMapApis({ resolution: null, width: null, height: null, data: null });
    render(<MapPanel telemetry={null} />);

    expect(await screen.findByText(/No map data yet/)).toBeInTheDocument();
  });

  it("shows the placeholder when data is an empty array", async () => {
    mockMapApis({ resolution: 1, width: 0, height: 0, data: [] });
    render(<MapPanel telemetry={null} />);

    await waitFor(() => expect(screen.getByText(/No map data yet/)).toBeInTheDocument());
  });

  it("renders a real map: dimensions text and a canvas, once real data arrives", async () => {
    const width = 3;
    const height = 3;
    mockMapApis({ resolution: 1, width, height, data: new Array(width * height).fill(0) });
    render(<MapPanel telemetry={null} />);

    expect(await screen.findByText(/3 × 3 cells/)).toBeInTheDocument();
    expect(screen.queryByText(/No map data yet/)).not.toBeInTheDocument();
    expect(await screen.findByTestId("map-canvas")).toBeInTheDocument();
  });

  it("shows the explored percentage from telemetry.mapping when available", async () => {
    const width = 2;
    const height = 2;
    mockMapApis({ resolution: 1, width, height, data: new Array(width * height).fill(0) });
    render(
      <MapPanel
        telemetry={{
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
          autonomy: { state: "SEARCHING_FRONTIER", objective: null, target: null, next_action: null },
          sensors: { slam: "ok", lidar: "ok", rangefinder: null, camera: null },
          mapping: {
            available: true, resolution_m: 1, width_cells: 2, height_cells: 2,
            origin_x: 0, origin_y: 0, coverage_cell_size_m: 1, explored_pct: 42.5,
          },
          navigation: { target: null, frontier_count: null, candidate_count: null, blacklisted_count: null, geofence_breached: null },
        }}
      />,
    );

    expect(await screen.findByText(/42\.5% searched/)).toBeInTheDocument();
  });
});
