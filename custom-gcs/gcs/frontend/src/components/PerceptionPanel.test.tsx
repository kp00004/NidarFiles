import { afterEach, describe, expect, it, vi } from "vitest";
import { render, screen } from "@testing-library/react";
import PerceptionPanel from "./PerceptionPanel";
import * as api from "../api";
import type { Detection, PerceptionDetectionsResponse, PerceptionStatusResponse } from "../types";

const BASE_STATUS: PerceptionStatusResponse = {
  camera_connected: true,
  detector_enabled: true,
  detector_ready: true,
  detector_backend: "dev-model",
  model_name: "yolov8n",
  person_count: 0,
  fps: 10,
  frame_width: 640,
  frame_height: 480,
  last_detection_age_s: 0.1,
};

const EMPTY_DETECTIONS: PerceptionDetectionsResponse = {
  frame_width: 640,
  frame_height: 480,
  timestamp: 1700000000,
  detections: [],
};

function makeDetection(overrides: Partial<Detection> = {}): Detection {
  return {
    detection_id: "det-1",
    class_name: "person",
    confidence: 0.91,
    bbox: { x_min: 10, y_min: 20, x_max: 100, y_max: 200 },
    center_x: 55,
    center_y: 110,
    track_id: "t1",
    source: "dev-model",
    model_name: "yolov8n",
    ...overrides,
  };
}

function mockPerceptionApis(status: PerceptionStatusResponse, detections: PerceptionDetectionsResponse) {
  vi.spyOn(api, "getPerceptionStatus").mockResolvedValue(status);
  vi.spyOn(api, "getPerceptionDetections").mockResolvedValue(detections);
}

afterEach(() => {
  vi.restoreAllMocks();
});

describe("PerceptionPanel", () => {
  it("shows CAMERA UNAVAILABLE when the camera isn't connected", async () => {
    mockPerceptionApis({ ...BASE_STATUS, camera_connected: false }, EMPTY_DETECTIONS);
    render(<PerceptionPanel />);

    expect(await screen.findByText(/CAMERA UNAVAILABLE/)).toBeInTheDocument();
  });

  it("shows PERCEPTION UNAVAILABLE when the camera is connected but the detector isn't ready", async () => {
    mockPerceptionApis({ ...BASE_STATUS, detector_ready: false }, EMPTY_DETECTIONS);
    render(<PerceptionPanel />);

    expect(await screen.findByText(/PERCEPTION UNAVAILABLE/)).toBeInTheDocument();
  });

  it("shows DETECTING... when ready but no person_count has been published yet", async () => {
    mockPerceptionApis({ ...BASE_STATUS, person_count: null }, EMPTY_DETECTIONS);
    render(<PerceptionPanel />);

    expect(await screen.findByText(/DETECTING\.\.\./)).toBeInTheDocument();
  });

  it("shows No detection when ready and the detections list is empty", async () => {
    mockPerceptionApis(BASE_STATUS, EMPTY_DETECTIONS);
    render(<PerceptionPanel />);

    expect(await screen.findByText(/No detection/)).toBeInTheDocument();
  });

  it("shows Person detected with the row's fields for a single detection, superseding the placeholder", async () => {
    mockPerceptionApis({ ...BASE_STATUS, person_count: 1 }, { ...EMPTY_DETECTIONS, detections: [makeDetection()] });
    render(<PerceptionPanel />);

    expect(await screen.findByText("Person detected")).toBeInTheDocument();
    expect(screen.getByText("det-1")).toBeInTheDocument();
    expect(screen.getByText(/91%/)).toBeInTheDocument();
    expect(screen.getByText(/\(10, 20\) – \(100, 200\)/)).toBeInTheDocument();
    expect(screen.queryByText(/No detection/)).not.toBeInTheDocument();
  });

  it("shows Multiple people (N) with all rows for more than one detection", async () => {
    mockPerceptionApis(
      { ...BASE_STATUS, person_count: 2 },
      {
        ...EMPTY_DETECTIONS,
        detections: [makeDetection({ detection_id: "det-1" }), makeDetection({ detection_id: "det-2", confidence: 0.5 })],
      },
    );
    render(<PerceptionPanel />);

    expect(await screen.findByText("Multiple people (2)")).toBeInTheDocument();
    expect(screen.getByText("det-1")).toBeInTheDocument();
    expect(screen.getByText("det-2")).toBeInTheDocument();
  });

  it("shows the standard error banner on fetch failure", async () => {
    vi.spyOn(api, "getPerceptionStatus").mockRejectedValue(new Error("network error"));
    vi.spyOn(api, "getPerceptionDetections").mockResolvedValue(EMPTY_DETECTIONS);
    render(<PerceptionPanel />);

    expect(await screen.findByText(/perception fetch failed: network error/)).toBeInTheDocument();
  });
});
