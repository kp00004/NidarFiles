import { afterEach, describe, expect, it, vi } from "vitest";
import { render, screen, waitFor } from "@testing-library/react";
import Footer from "./Footer";
import * as api from "../api";

// Footer is the one place the frontend surfaces GET /health's ros_status
// (see gcs/backend/app/schemas.py's HealthResponse docstring and
// src/types.ts's RosStatus) -- reusing the existing rosbridge-target poll
// rather than a second status mechanism. Covers all three backend states
// this task added: "connected" (unchanged Jetson-deployment behavior),
// "disabled" (GCS_ROS_ENABLED=false, ROS-optional local development), and
// "unavailable" (ROS enabled but rosbridge unreachable).

afterEach(() => {
  vi.restoreAllMocks();
});

describe("Footer", () => {
  it("shows the rosbridge target and 'connected' when ROS is connected", async () => {
    vi.spyOn(api, "getHealth").mockResolvedValue({
      connected: true,
      ros_status: "connected",
      rosbridge_host: "127.0.0.1",
      rosbridge_port: 9090,
    });

    render(<Footer />);

    await waitFor(() =>
      expect(screen.getByText(/rosbridge target: 127\.0\.0\.1:9090/)).toBeInTheDocument(),
    );
    expect(screen.getByText(/ROS: connected/)).toBeInTheDocument();
  });

  it("labels ROS-optional local-development mode when the backend reports 'disabled'", async () => {
    vi.spyOn(api, "getHealth").mockResolvedValue({
      connected: false,
      ros_status: "disabled",
      rosbridge_host: "127.0.0.1",
      rosbridge_port: 9090,
    });

    render(<Footer />);

    const label = await screen.findByText(/ROS disabled \(GCS_ROS_ENABLED=false\)/);
    expect(label).toBeInTheDocument();
    // Flagged visually (text-warn), same convention as
    // ConnectionPanel.tsx's stale-heartbeat warning -- never "disabled"
    // reads as green in the UI.
    expect(label.closest("span")?.className).toMatch(/text-warn/);
  });

  it("labels an unreachable rosbridge as 'unavailable', distinct from 'disabled'", async () => {
    vi.spyOn(api, "getHealth").mockResolvedValue({
      connected: false,
      ros_status: "unavailable",
      rosbridge_host: "10.0.0.5",
      rosbridge_port: 9090,
    });

    render(<Footer />);

    const label = await screen.findByText(/ROS unavailable/);
    expect(label).toBeInTheDocument();
    expect(label.closest("span")?.className).toMatch(/text-warn/);
  });

  it("names the command radio as the telemetry source when the backend uses it", async () => {
    vi.spyOn(api, "getHealth").mockResolvedValue({
      connected: true,
      ros_status: "disabled",
      telemetry_source: "radio",
      rosbridge_host: "127.0.0.1",
      rosbridge_port: 9090,
    });

    render(<Footer />);

    expect(await screen.findByText(/telemetry: MicroLR900 command radio/)).toBeInTheDocument();
    expect(screen.queryByText(/ROS disabled/)).not.toBeInTheDocument();
  });

  it("falls back to 'unknown' for both target and ROS status if the health fetch fails", async () => {
    vi.spyOn(api, "getHealth").mockRejectedValue(new Error("network error"));

    render(<Footer />);

    await waitFor(() => expect(screen.getByText(/rosbridge target: unknown/)).toBeInTheDocument());
    expect(screen.getByText(/ROS: unknown/)).toBeInTheDocument();
  });
});
