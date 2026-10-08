import { describe, expect, it } from "vitest";
import { render, screen } from "@testing-library/react";
import GpsPanel from "./GpsPanel";
import type { TelemetryResponse } from "../types";

function telemetryWith(gps: TelemetryResponse["gps"]): TelemetryResponse {
  return { gps } as TelemetryResponse;
}

describe("GpsPanel", () => {
  it("says it is for GPS missions only", () => {
    render(<GpsPanel telemetry={null} />);
    expect(screen.getByText(/GPS missions only/)).toBeInTheDocument();
  });

  it("shows fix, satellites and position when GPS data is present", () => {
    render(
      <GpsPanel telemetry={telemetryWith({ fix_status: 3, satellites_visible: 11, latitude: 12.9715987, longitude: 77.5945627, altitude: 920.4 })} />,
    );
    expect(screen.getByText(/^3 \(/)).toBeInTheDocument();
    expect(screen.getByText("11")).toBeInTheDocument();
    expect(screen.getByText("12.9715987°")).toBeInTheDocument();
    expect(screen.getByText("77.5945627°")).toBeInTheDocument();
    expect(screen.getByText("920.4 m")).toBeInTheDocument();
  });

  it("shows unavailable without GPS (the indoor drone)", () => {
    render(<GpsPanel telemetry={telemetryWith(null)} />);
    expect(screen.getAllByText("unavailable")).toHaveLength(5);
  });
});
