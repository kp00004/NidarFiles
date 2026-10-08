import { describe, expect, it } from "vitest";
import { render, screen } from "@testing-library/react";
import StatusTextPanel, { collapse } from "./StatusTextPanel";
import type { TelemetryResponse } from "../types";

function withMessages(statustext: TelemetryResponse["statustext"]): TelemetryResponse {
  return { statustext } as TelemetryResponse;
}

describe("StatusTextPanel", () => {
  it("labels severity in words and explains known messages", () => {
    render(
      <StatusTextPanel
        telemetry={withMessages([
          { severity: 4, text: "EKF3 IMU0 MAG0 ground mag anomaly, yaw re-aligned" },
          { severity: 2, text: "PreArm: Check mag field (xy diff:150>100)" },
        ])}
      />,
    );
    expect(screen.getByText("CRITICAL")).toBeInTheDocument();
    expect(screen.getByText("WARNING")).toBeInTheDocument();
    expect(screen.getByText(/calibrate the compass/)).toBeInTheDocument();
    expect(screen.queryByText(/\[sev/)).not.toBeInTheDocument();
  });

  it("shows newest first and collapses repeats", () => {
    const rows = collapse([
      { severity: 6, text: "old" },
      { severity: 4, text: "stopped aiding" },
      { severity: 4, text: "stopped aiding" },
      { severity: 4, text: "stopped aiding" },
    ]);
    expect(rows.map((r) => [r.text, r.count])).toEqual([["stopped aiding", 3], ["old", 1]]);
  });

  it("says when there are no messages", () => {
    render(<StatusTextPanel telemetry={null} />);
    expect(screen.getByText("no messages received")).toBeInTheDocument();
  });
});
