import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import { act, fireEvent, render, screen, waitFor } from "@testing-library/react";
import ControlsPanel from "./ControlsPanel";
import * as api from "../api";
import type { RadioCommandResponse, RadioStatusResponse } from "../types";

const HOVER = { id: "hover", name: "Hover", description: "real hover" };
const MOTOR_TEST = { id: "motor_test", name: "Motor Test", description: "props off" };

const RADIO_UP: RadioStatusResponse = {
  enabled: true,
  port: "COM5",
  baud: 115200,
  port_open: true,
  port_error: null,
  jetson_link_up: true,
  jetson_heartbeat_age_s: 0.4,
  jetson_mission_state: "idle",
  last_command: null,
};

function accepted(command: "start" | "abort"): RadioCommandResponse {
  return { status: "accepted", command, mission: command === "start" ? "hover" : null, nonce: 7, attempts: 1 };
}

beforeEach(() => {
  vi.spyOn(api, "getMissions").mockResolvedValue([HOVER, MOTOR_TEST]);
  vi.spyOn(api, "getRadioStatus").mockResolvedValue(RADIO_UP);
  vi.spyOn(api, "getFlightTestStatus").mockResolvedValue({
    scenario: "hover", state: "idle", target_altitude_m: 0.5, current_altitude_m: null,
    current_position: null, duration_s: 10, elapsed_hover_s: null, armed: false,
    execution_mode: "real", detail: "waiting for START", flight_mode: "STABILIZE",
  });
});

afterEach(() => {
  vi.restoreAllMocks();
});

async function renderReady() {
  render(<ControlsPanel telemetry={null} />);
  await screen.findByRole("option", { name: "Hover" });
}

describe("ControlsPanel (Mission Control)", () => {
  it("shows a Mission dropdown with Hover and Motor Test, Hover selected by default", async () => {
    await renderReady();
    const select = screen.getByRole("combobox", { name: "Mission" }) as HTMLSelectElement;
    expect(screen.getAllByRole("option").map((o) => o.textContent)).toEqual(["Hover", "Motor Test"]);
    expect(select.value).toBe("hover");
    expect(screen.getByText(/REAL FLIGHT\. START \(Hover\)/)).toBeInTheDocument();
  });

  it("selecting Motor Test sends its id on START and shows the props-off warning", async () => {
    const spy = vi.spyOn(api, "postMissionStart").mockResolvedValue({ ...accepted("start"), mission: "motor_test" });
    await renderReady();
    fireEvent.change(screen.getByRole("combobox", { name: "Mission" }), { target: { value: "motor_test" } });
    expect(screen.getByText(/PROPS OFF\. START \(Motor Test\)/)).toBeInTheDocument();
    await act(async () => {
      fireEvent.click(screen.getByRole("button", { name: "START" }));
    });
    expect(spy).toHaveBeenCalledWith("motor_test");
  });

  it("names the active mission next to its radio state", async () => {
    vi.spyOn(api, "getRadioStatus").mockResolvedValue({ ...RADIO_UP, jetson_mission: "motor_test", jetson_mission_state: "testing" });
    await renderReady();
    expect(await screen.findByText("testing")).toBeInTheDocument();
    // once in the Mission dropdown, once in the status line
    expect(screen.getAllByText("Motor Test")).toHaveLength(2);
  });

  it("START sends the selected mission id", async () => {
    const spy = vi.spyOn(api, "postMissionStart").mockResolvedValue(accepted("start"));
    await renderReady();
    await act(async () => {
      fireEvent.click(screen.getByRole("button", { name: "START" }));
    });
    expect(spy).toHaveBeenCalledWith("hover");
    expect(await screen.findByText(/Jetson ACCEPTED START \(hover\)/)).toBeInTheDocument();
  });

  it("ABORT calls postAbort", async () => {
    const spy = vi.spyOn(api, "postAbort").mockResolvedValue(accepted("abort"));
    await renderReady();
    await act(async () => {
      fireEvent.click(screen.getByRole("button", { name: "STOP / ABORT" }));
    });
    expect(spy).toHaveBeenCalled();
    expect(await screen.findByText(/Jetson ACCEPTED ABORT/)).toBeInTheDocument();
  });

  it("a rejected START is shown as not accepted, with the Jetson's reason", async () => {
    vi.spyOn(api, "postMissionStart").mockRejectedValue(
      new Error("POST /api/mission/start failed: HTTP 409: Jetson REJECTED START: FCU_NOT_CONNECTED"),
    );
    await renderReady();
    await act(async () => {
      fireEvent.click(screen.getByRole("button", { name: "START" }));
    });
    expect(await screen.findByText(/START not accepted: .*FCU_NOT_CONNECTED/)).toBeInTheDocument();
    expect(screen.queryByText(/ACCEPTED START/)).not.toBeInTheDocument();
  });

  it("never disables ABORT because of a pending START", async () => {
    let resolveStart: (v: RadioCommandResponse) => void = () => {};
    vi.spyOn(api, "postMissionStart").mockReturnValue(new Promise((r) => (resolveStart = r)));
    const abortSpy = vi.spyOn(api, "postAbort").mockResolvedValue(accepted("abort"));
    await renderReady();

    const startBtn = screen.getByRole("button", { name: "START" });
    const abortBtn = screen.getByRole("button", { name: "STOP / ABORT" });
    fireEvent.click(startBtn);
    await waitFor(() => expect(startBtn).toBeDisabled());

    expect(abortBtn).not.toBeDisabled();
    await act(async () => {
      fireEvent.click(abortBtn);
    });
    expect(abortSpy).toHaveBeenCalled();

    await act(async () => {
      resolveStart(accepted("start"));
    });
    await waitFor(() => expect(startBtn).not.toBeDisabled());
  });

  it("a failed ABORT tells the operator to use the RC transmitter", async () => {
    vi.spyOn(api, "postAbort").mockRejectedValue(new Error("HTTP 504"));
    await renderReady();
    await act(async () => {
      fireEvent.click(screen.getByRole("button", { name: "STOP / ABORT" }));
    });
    expect(await screen.findByText(/ABORT not confirmed: HTTP 504 — USE THE RC TRANSMITTER/)).toBeInTheDocument();
  });

  it("shows RADIO LINK DOWN when no Jetson heartbeat arrives", async () => {
    vi.spyOn(api, "getRadioStatus").mockResolvedValue({ ...RADIO_UP, jetson_link_up: false, jetson_heartbeat_age_s: 9 });
    await renderReady();
    expect(await screen.findByText(/RADIO LINK DOWN/)).toBeInTheDocument();
  });

  it("shows a closed radio port with its error", async () => {
    vi.spyOn(api, "getRadioStatus").mockResolvedValue({
      ...RADIO_UP, port_open: false, jetson_link_up: false, port_error: "could not open port 'COM5'",
    });
    await renderReady();
    expect(await screen.findByText(/PORT COM5 NOT OPEN/)).toBeInTheDocument();
  });

  it("has exactly two buttons -- no confirmation step before ABORT", async () => {
    await renderReady();
    expect(screen.getAllByRole("button")).toHaveLength(2);
  });

  it("shows one readable status block: link, mission with state badge, detail, vehicle", async () => {
    vi.spyOn(api, "getRadioStatus").mockResolvedValue({ ...RADIO_UP, jetson_mission: "hover", jetson_mission_state: "aborted" });
    vi.spyOn(api, "getFlightTestStatus").mockResolvedValue({
      scenario: "hover", state: "aborted", target_altitude_m: 0.5, current_altitude_m: null,
      current_position: null, duration_s: 10, elapsed_hover_s: null, armed: false,
      execution_mode: "real", detail: "landed and disarmed after ABORT", flight_mode: "LAND",
    });
    render(<ControlsPanel telemetry={null} />);
    expect(await screen.findByText("aborted")).toBeInTheDocument();
    expect(screen.getByText("landed and disarmed after ABORT")).toBeInTheDocument();
    expect(screen.getByText(/LINK UP · heartbeat 0.4 s ago/)).toBeInTheDocument();
    expect(screen.getByText("Pixhawk not connected (no FCU telemetry)")).toBeInTheDocument();
  });
});
