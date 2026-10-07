import { afterEach, describe, expect, it, vi } from "vitest";
import { render, screen } from "@testing-library/react";
import SurvivorsPanel from "./SurvivorsPanel";
import * as api from "../api";

afterEach(() => {
  vi.restoreAllMocks();
});

describe("SurvivorsPanel", () => {
  it("shows the explicit not-implemented placeholder when the list is empty", async () => {
    vi.spyOn(api, "getSurvivors").mockResolvedValue([]);
    render(<SurvivorsPanel />);

    expect(
      await screen.findByText(/No survivors detected yet — detection not implemented in onboard-autonomy\./),
    ).toBeInTheDocument();
  });

  it("renders real rows instead of the placeholder once survivors arrive", async () => {
    vi.spyOn(api, "getSurvivors").mockResolvedValue([
      { survivor_id: 1, x: 2.5, y: 3.5, confidence: 0.8 },
    ]);
    render(<SurvivorsPanel />);

    expect(await screen.findByText("#1")).toBeInTheDocument();
    expect(screen.queryByText(/No survivors detected yet/)).not.toBeInTheDocument();
  });
});
