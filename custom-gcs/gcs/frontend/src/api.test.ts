import { afterEach, describe, expect, it, vi } from "vitest";
import {
  getCameraStatus,
  getFlightTestStatus,
  getMissions,
  getMultiStepFlightTestStatus,
  getPerceptionDetections,
  getPerceptionStatus,
  getRadioStatus,
  getTelemetry,
  postAbort,
  postMissionStart,
} from "./api";

// Direct replacement for the old prototype's raw-HTML string-matching
// test -- verifies api.ts calls exactly the documented root-relative
// paths, with no host, matching gcs/backend/app/schemas.py's contract.

function mockFetchOnce(body: unknown, ok = true, status = 200) {
  const fetchMock = vi.fn().mockResolvedValue({
    ok,
    status,
    json: () => Promise.resolve(body),
  });
  vi.stubGlobal("fetch", fetchMock);
  return fetchMock;
}

afterEach(() => {
  vi.unstubAllGlobals();
});

describe("api.ts", () => {
  it("getTelemetry calls exactly /api/telemetry", async () => {
    const fetchMock = mockFetchOnce({ connected: true });
    await getTelemetry();
    expect(fetchMock).toHaveBeenCalledTimes(1);
    expect(fetchMock).toHaveBeenCalledWith("/api/telemetry");
  });

  it("postAbort calls exactly POST /api/command/abort", async () => {
    const fetchMock = mockFetchOnce({ status: "accepted", command: "abort", mission: null, nonce: 1, attempts: 1 });
    await postAbort();
    expect(fetchMock).toHaveBeenCalledTimes(1);
    const [url, init] = fetchMock.mock.calls[0];
    expect(url).toBe("/api/command/abort");
    expect(init).toMatchObject({ method: "POST" });
    expect(init.signal).toBeInstanceOf(AbortSignal);
  });

  it("getRadioStatus calls exactly /api/radio/status", async () => {
    const fetchMock = mockFetchOnce({ enabled: true, port_open: true, jetson_link_up: true });
    await getRadioStatus();
    expect(fetchMock).toHaveBeenCalledWith("/api/radio/status");
  });

  it("getPerceptionDetections calls exactly /api/perception/detections", async () => {
    const fetchMock = mockFetchOnce({ frame_width: null, frame_height: null, timestamp: null, detections: [] });
    await getPerceptionDetections();
    expect(fetchMock).toHaveBeenCalledTimes(1);
    expect(fetchMock).toHaveBeenCalledWith("/api/perception/detections");
  });

  it("getPerceptionStatus calls exactly /api/perception/status", async () => {
    const fetchMock = mockFetchOnce({
      camera_connected: null,
      detector_enabled: null,
      detector_ready: null,
      detector_backend: null,
      model_name: null,
      person_count: null,
      fps: null,
      frame_width: null,
      frame_height: null,
      last_detection_age_s: null,
    });
    await getPerceptionStatus();
    expect(fetchMock).toHaveBeenCalledTimes(1);
    expect(fetchMock).toHaveBeenCalledWith("/api/perception/status");
  });

  it("getCameraStatus calls exactly /api/camera/status", async () => {
    const fetchMock = mockFetchOnce({ connected: null, stream_url: null, frame_width: null, frame_height: null, fps: null });
    await getCameraStatus();
    expect(fetchMock).toHaveBeenCalledTimes(1);
    expect(fetchMock).toHaveBeenCalledWith("/api/camera/status");
  });

  it("getMissions calls exactly /api/missions", async () => {
    const fetchMock = mockFetchOnce([]);
    await getMissions();
    expect(fetchMock).toHaveBeenCalledTimes(1);
    expect(fetchMock).toHaveBeenCalledWith("/api/missions");
  });

  it("getFlightTestStatus calls exactly /api/flight-test/status", async () => {
    const fetchMock = mockFetchOnce({
      scenario: null, state: null, target_altitude_m: null, current_altitude_m: null,
      current_position: null, duration_s: null, elapsed_hover_s: null, armed: null, execution_mode: null,
    });
    await getFlightTestStatus();
    expect(fetchMock).toHaveBeenCalledTimes(1);
    expect(fetchMock).toHaveBeenCalledWith("/api/flight-test/status");
  });

  it("getMultiStepFlightTestStatus calls exactly /api/flight-test/multi-step/status", async () => {
    const fetchMock = mockFetchOnce({
      scenario_id: null, state: null, phase: null, current_step_index: null, current_step_action: null,
      total_steps: null, current_position: null, armed: null, execution_mode: null,
    });
    await getMultiStepFlightTestStatus();
    expect(fetchMock).toHaveBeenCalledTimes(1);
    expect(fetchMock).toHaveBeenCalledWith("/api/flight-test/multi-step/status");
  });

  it("postMissionStart posts exactly {mission} to /api/mission/start", async () => {
    const fetchMock = mockFetchOnce({ status: "accepted", command: "start", mission: "hover", nonce: 1, attempts: 1 });
    await postMissionStart("hover");
    expect(fetchMock).toHaveBeenCalledTimes(1);
    const [url, init] = fetchMock.mock.calls[0];
    expect(url).toBe("/api/mission/start");
    expect(init).toMatchObject({ method: "POST" });
    expect(JSON.parse(init.body as string)).toEqual({ mission: "hover" });
    expect(init.signal).toBeInstanceOf(AbortSignal);
  });

  it("no call ever uses an absolute URL with a host", async () => {
    const fetchMock = mockFetchOnce({ status: "accepted" });
    await postMissionStart("hover");
    const [url] = fetchMock.mock.calls[0];
    expect(url).not.toMatch(/^https?:\/\//);
    expect((url as string).startsWith("/")).toBe(true);
  });

  it("rejects with a clear error on a non-OK HTTP status", async () => {
    mockFetchOnce({}, false, 503);
    await expect(getTelemetry()).rejects.toThrow(/503/);
  });

  it("postAbort surfaces the backend's structured detail message on a non-OK status", async () => {
    mockFetchOnce({ detail: "radio port COM5 is not open -- ABORT not sent" }, false, 503);
    await expect(postAbort()).rejects.toThrow(
      "POST /api/command/abort failed: HTTP 503: radio port COM5 is not open -- ABORT not sent",
    );
  });

  it("postMissionStart surfaces a Jetson rejection (409) instead of resolving", async () => {
    mockFetchOnce({ detail: "Jetson REJECTED START: MISSION_NOT_READY" }, false, 409);
    await expect(postMissionStart("hover")).rejects.toThrow(/409: Jetson REJECTED START: MISSION_NOT_READY/);
  });

  it("postAbort falls back to a bare HTTP status when the error body has no detail field", async () => {
    mockFetchOnce({}, false, 500);
    await expect(postAbort()).rejects.toThrow("POST /api/command/abort failed: HTTP 500");
  });

  it("postMissionStart rejects with a clear timeout error at 5000ms instead of hanging forever", async () => {
    vi.useFakeTimers();
    // A fetch that never resolves on its own, but honours the
    // AbortSignal like a real fetch implementation would -- this is what
    // lets the timeout actually unblock the caller.
    const fetchMock = vi.fn((_path: string, init?: RequestInit) => {
      return new Promise((_resolve, reject) => {
        init?.signal?.addEventListener("abort", () => {
          reject(new DOMException("The operation was aborted.", "AbortError"));
        });
      });
    });
    vi.stubGlobal("fetch", fetchMock);

    const pending = expect(postMissionStart("hover")).rejects.toThrow(/timed out after 5000ms/);
    await vi.advanceTimersByTimeAsync(5000);
    await pending;

    vi.useRealTimers();
  });

  it("postAbort times out sooner (3000ms), not the 5000ms START bound", async () => {
    vi.useFakeTimers();
    const fetchMock = vi.fn((_path: string, init?: RequestInit) => {
      return new Promise((_resolve, reject) => {
        init?.signal?.addEventListener("abort", () => {
          reject(new DOMException("The operation was aborted.", "AbortError"));
        });
      });
    });
    vi.stubGlobal("fetch", fetchMock);

    const pending = expect(postAbort()).rejects.toThrow(/timed out after 3000ms/);
    await vi.advanceTimersByTimeAsync(3000);
    await pending;

    vi.useRealTimers();
  });

  it("does not abort postAbort early -- it still hasn't fired by 2500ms (covers radio retries)", async () => {
    vi.useFakeTimers();
    let aborted = false;
    const fetchMock = vi.fn((_path: string, init?: RequestInit) => {
      return new Promise(() => {
        init?.signal?.addEventListener("abort", () => {
          aborted = true;
        });
      });
    });
    vi.stubGlobal("fetch", fetchMock);

    postAbort().catch(() => {});
    await vi.advanceTimersByTimeAsync(2500);
    expect(aborted).toBe(false);

    vi.useRealTimers();
  });
});
