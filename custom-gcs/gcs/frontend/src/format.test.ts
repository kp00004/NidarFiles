import { describe, expect, it } from "vitest";
import { fmtNum, fmtOrUnavailable, gpsFixLabel, mavStateLabel } from "./format";

// mavStateLabel/gpsFixLabel exist because bare MAV_STATE/GPS_FIX_TYPE
// integers were a real source of operator confusion (a `system_status`
// of 5 looked alarming until traced to MAV_STATE_CRITICAL) — see
// CHECKPOINT/CURRENT_STATE.md §17.

describe("fmtNum", () => {
  it("formats a known number", () => {
    expect(fmtNum(1.2345)).toBe("1.23");
  });

  it("falls back to em dash for null/undefined", () => {
    expect(fmtNum(null)).toBe("—");
    expect(fmtNum(undefined)).toBe("—");
  });
});

describe("fmtOrUnavailable", () => {
  it("formats a known number with suffix", () => {
    expect(fmtOrUnavailable(3, " m")).toBe("3.00 m");
  });

  it("falls back to 'unavailable' for null/undefined", () => {
    expect(fmtOrUnavailable(null)).toBe("unavailable");
    expect(fmtOrUnavailable(undefined)).toBe("unavailable");
  });
});

describe("mavStateLabel", () => {
  it("labels a known MAV_STATE code", () => {
    expect(mavStateLabel(5)).toBe("CRITICAL");
  });

  it("labels an unrecognized code as UNKNOWN(<code>)", () => {
    expect(mavStateLabel(99)).toBe("UNKNOWN(99)");
  });

  it("returns an em dash for null/undefined", () => {
    expect(mavStateLabel(null)).toBe("—");
    expect(mavStateLabel(undefined)).toBe("—");
  });
});

describe("gpsFixLabel", () => {
  it("labels a known GPS_FIX_TYPE code", () => {
    expect(gpsFixLabel(3)).toBe("FIX_3D");
  });

  it("labels an unrecognized code as UNKNOWN(<code>)", () => {
    expect(gpsFixLabel(42)).toBe("UNKNOWN(42)");
  });

  it("returns 'unavailable' for null/undefined", () => {
    expect(gpsFixLabel(null)).toBe("unavailable");
    expect(gpsFixLabel(undefined)).toBe("unavailable");
  });
});
