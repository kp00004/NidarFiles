import { afterEach, describe, expect, it, vi } from "vitest";
import { act, render, screen, waitFor } from "@testing-library/react";
import { useTelemetry } from "./useTelemetry";
import * as api from "../api";
import type { TelemetryResponse } from "../types";

const GOOD_TELEMETRY: TelemetryResponse = {
  connected: true,
  mission_state: "idle",
  fcu: { connected: true, armed: false, guided: false, mode: "GUIDED", system_status: 3 },
  battery: { voltage: 12.1, current: 1.2, percentage: 0.9 },
  pose: { position: { x: 1, y: 2, z: 3 } },
  velocity: { x: 0, y: 0, z: 0 },
  attitude: { x: 0, y: 0, z: 0, w: 1 },
  gps: { fix_status: 3, satellites_visible: 8, latitude: 0, longitude: 0, altitude: 0 },
  statustext: [],
  heartbeat_age_s: 0.2,
  autonomy: { state: null, objective: null, target: null, next_action: null },
  sensors: { slam: null, lidar: null, rangefinder: null, camera: null },
  mapping: {
    available: false, resolution_m: null, width_cells: null, height_cells: null,
    origin_x: null, origin_y: null, coverage_cell_size_m: null, explored_pct: null,
  },
  navigation: { target: null, frontier_count: null, candidate_count: null, blacklisted_count: null, geofence_breached: null },
};

function Probe() {
  const { data, error, lastUpdatedAt } = useTelemetry();
  return (
    <div>
      <div data-testid="connected">{data ? String(data.connected) : "null"}</div>
      <div data-testid="error">{error ?? "none"}</div>
      <div data-testid="updated">{lastUpdatedAt ? "yes" : "no"}</div>
    </div>
  );
}

afterEach(() => {
  vi.restoreAllMocks();
  vi.useRealTimers();
});

describe("useTelemetry", () => {
  it("renders a normal state when the fetch succeeds", async () => {
    vi.spyOn(api, "getTelemetry").mockResolvedValue(GOOD_TELEMETRY);
    render(<Probe />);

    await waitFor(() => expect(screen.getByTestId("connected")).toHaveTextContent("true"));
    expect(screen.getByTestId("error")).toHaveTextContent("none");
    expect(screen.getByTestId("updated")).toHaveTextContent("yes");
  });

  it("surfaces an error state when telemetry fetch fails, without going blank", async () => {
    vi.spyOn(api, "getTelemetry").mockRejectedValue(new Error("HTTP 503"));
    render(<Probe />);

    await waitFor(() => expect(screen.getByTestId("error")).toHaveTextContent("HTTP 503"));
    expect(screen.getByTestId("connected")).toHaveTextContent("null");
  });

  it("keeps the last good data visible after a later fetch failure", async () => {
    const spy = vi
      .spyOn(api, "getTelemetry")
      .mockResolvedValueOnce(GOOD_TELEMETRY)
      .mockRejectedValueOnce(new Error("network error"));

    vi.useFakeTimers({ shouldAdvanceTime: true });
    render(<Probe />);

    await waitFor(() => expect(screen.getByTestId("connected")).toHaveTextContent("true"));

    await act(async () => {
      vi.advanceTimersByTime(1000);
    });

    await waitFor(() => expect(screen.getByTestId("error")).toHaveTextContent("network error"));
    expect(screen.getByTestId("connected")).toHaveTextContent("true");
    expect(spy).toHaveBeenCalledTimes(2);
  });

  it("clears a prior error once a later poll succeeds again (reconnection)", async () => {
    // State x event this repo's own bar (see test_state_machine.py's
    // pattern) requires covering: error -> success, not just success and
    // success -> error. This is what the operator sees on a rosbridge
    // reconnect after a drop -- if `error` were never cleared on
    // success, the UI would keep showing a stale "telemetry fetch
    // failed" banner over live data forever.
    const spy = vi
      .spyOn(api, "getTelemetry")
      .mockRejectedValueOnce(new Error("network error"))
      .mockResolvedValueOnce(GOOD_TELEMETRY);

    vi.useFakeTimers({ shouldAdvanceTime: true });
    render(<Probe />);

    await waitFor(() => expect(screen.getByTestId("error")).toHaveTextContent("network error"));
    expect(screen.getByTestId("connected")).toHaveTextContent("null");

    await act(async () => {
      vi.advanceTimersByTime(1000);
    });

    await waitFor(() => expect(screen.getByTestId("error")).toHaveTextContent("none"));
    expect(screen.getByTestId("connected")).toHaveTextContent("true");
    expect(spy).toHaveBeenCalledTimes(2);
  });
});
