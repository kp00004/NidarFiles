import { afterEach, describe, expect, it, vi } from "vitest";
import { act, fireEvent, render, screen } from "@testing-library/react";
import SetupPanel from "./SetupPanel";
import * as api from "../api";

afterEach(() => {
  vi.restoreAllMocks();
});

describe("SetupPanel", () => {
  it("reads a parameter and fills in its value", async () => {
    const spy = vi.spyOn(api, "getParam").mockResolvedValue({ name: "RNGFND1_MAX_CM", value: 8, attempts: 1 });
    render(<SetupPanel />);
    fireEvent.change(screen.getByLabelText("Parameter name"), { target: { value: "rngfnd1_max_cm" } });
    await act(async () => {
      fireEvent.click(screen.getByRole("button", { name: "Read" }));
    });
    expect(spy).toHaveBeenCalledWith("RNGFND1_MAX_CM");
    expect(screen.getByText("RNGFND1_MAX_CM = 8")).toBeInTheDocument();
    expect((screen.getByLabelText("Parameter value") as HTMLInputElement).value).toBe("8");
  });

  it("writes and shows the value read back from the FCU", async () => {
    const spy = vi.spyOn(api, "setParam").mockResolvedValue({ name: "RNGFND1_MAX_CM", value: 800, attempts: 1 });
    render(<SetupPanel />);
    fireEvent.change(screen.getByLabelText("Parameter name"), { target: { value: "RNGFND1_MAX_CM" } });
    fireEvent.change(screen.getByLabelText("Parameter value"), { target: { value: "800" } });
    await act(async () => {
      fireEvent.click(screen.getByRole("button", { name: "Write" }));
    });
    expect(spy).toHaveBeenCalledWith("RNGFND1_MAX_CM", 800);
    expect(screen.getByText(/WROTE RNGFND1_MAX_CM = 800/)).toBeInTheDocument();
  });

  it("shows the Jetson's refusal", async () => {
    vi.spyOn(api, "setParam").mockRejectedValue(new Error("write BATT_ARM_VOLT refused: vehicle armed"));
    render(<SetupPanel />);
    fireEvent.change(screen.getByLabelText("Parameter name"), { target: { value: "BATT_ARM_VOLT" } });
    fireEvent.change(screen.getByLabelText("Parameter value"), { target: { value: "13" } });
    await act(async () => {
      fireEvent.click(screen.getByRole("button", { name: "Write" }));
    });
    expect(screen.getByText(/vehicle armed/)).toBeInTheDocument();
  });

  it("does not write a non-number", () => {
    const spy = vi.spyOn(api, "setParam");
    render(<SetupPanel />);
    fireEvent.change(screen.getByLabelText("Parameter name"), { target: { value: "BATT_ARM_VOLT" } });
    fireEvent.change(screen.getByLabelText("Parameter value"), { target: { value: "abc" } });
    expect(screen.getByRole("button", { name: "Write" })).toBeDisabled();
    expect(spy).not.toHaveBeenCalled();
  });

  it("quick buttons read that parameter", async () => {
    const spy = vi.spyOn(api, "getParam").mockResolvedValue({ name: "ARMING_CHECK", value: 1, attempts: 1 });
    render(<SetupPanel />);
    await act(async () => {
      fireEvent.click(screen.getByRole("button", { name: "ARMING_CHECK" }));
    });
    expect(spy).toHaveBeenCalledWith("ARMING_CHECK");
  });
});
