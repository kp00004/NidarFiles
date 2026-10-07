import { afterEach, describe, expect, it, vi } from "vitest";
import { fireEvent, render, screen, waitFor } from "@testing-library/react";
import CameraPanel from "./CameraPanel";
import * as api from "../api";
import * as overlayRenderModule from "../overlayRender";
import type { CameraStatusResponse, Detection, PerceptionDetectionsResponse } from "../types";

const EMPTY_DETECTIONS: PerceptionDetectionsResponse = {
  frame_width: null,
  frame_height: null,
  timestamp: null,
  detections: [],
};

function mockCameraApis(camera: CameraStatusResponse, detections: PerceptionDetectionsResponse = EMPTY_DETECTIONS) {
  vi.spyOn(api, "getCameraStatus").mockResolvedValue(camera);
  vi.spyOn(api, "getPerceptionDetections").mockResolvedValue(detections);
}

afterEach(() => {
  vi.restoreAllMocks();
});

describe("CameraPanel", () => {
  it("shows CAMERA DISCONNECTED and renders no <img> when the camera isn't connected", async () => {
    mockCameraApis({ connected: false, stream_url: null, frame_width: null, frame_height: null, fps: null });
    render(<CameraPanel />);

    expect(await screen.findByText(/CAMERA DISCONNECTED/)).toBeInTheDocument();
    expect(screen.queryByRole("img")).not.toBeInTheDocument();
  });

  it("shows a distinct no-stream placeholder when connected but stream_url is missing", async () => {
    mockCameraApis({ connected: true, stream_url: null, frame_width: null, frame_height: null, fps: null });
    render(<CameraPanel />);

    expect(await screen.findByText(/CAMERA CONNECTED — NO VIDEO STREAM/)).toBeInTheDocument();
    expect(screen.queryByRole("img")).not.toBeInTheDocument();
  });

  it("renders an <img> with the reported stream_url once connected with a stream, and clears the placeholders", async () => {
    mockCameraApis({
      connected: true,
      stream_url: "http://10.0.0.5:8090/stream.mjpg",
      frame_width: 640,
      frame_height: 480,
      fps: 15,
    });
    render(<CameraPanel />);

    const img = await screen.findByRole("img");
    expect(img).toHaveAttribute("src", "http://10.0.0.5:8090/stream.mjpg");
    expect(screen.queryByText(/CAMERA DISCONNECTED/)).not.toBeInTheDocument();
    expect(screen.queryByText(/NO VIDEO STREAM/)).not.toBeInTheDocument();
    expect(await screen.findByText(/● LIVE/)).toBeInTheDocument();
  });

  it("shows the standard error banner on fetch failure", async () => {
    vi.spyOn(api, "getCameraStatus").mockRejectedValue(new Error("network error"));
    vi.spyOn(api, "getPerceptionDetections").mockResolvedValue(EMPTY_DETECTIONS);
    render(<CameraPanel />);

    expect(await screen.findByText(/camera fetch failed: network error/)).toBeInTheDocument();
  });

  it("switches to an OFFLINE indicator when the <img> itself fails to load (browser can't reach the Jetson)", async () => {
    mockCameraApis({
      connected: true,
      stream_url: "http://10.0.0.5:8090/stream.mjpg",
      frame_width: 640,
      frame_height: 480,
      fps: 15,
    });
    render(<CameraPanel />);

    const img = await screen.findByRole("img");
    expect(await screen.findByText(/● LIVE/)).toBeInTheDocument();

    fireEvent.error(img);

    expect(await screen.findByText(/● OFFLINE/)).toBeInTheDocument();
    expect(screen.queryByText(/● LIVE/)).not.toBeInTheDocument();
  });

  it("wires real detections and the actual rendered display size into the overlay renderer once the stream loads", async () => {
    const detections: Detection[] = [
      {
        detection_id: "det-1",
        class_name: "person",
        confidence: 0.9,
        bbox: { x_min: 10, y_min: 20, x_max: 100, y_max: 200 },
        center_x: 55,
        center_y: 110,
        track_id: null,
        source: "dev-model",
        model_name: "yolov8n",
      },
      {
        detection_id: "det-2",
        class_name: "person",
        confidence: 0.7,
        bbox: { x_min: 300, y_min: 40, x_max: 380, y_max: 260 },
        center_x: 340,
        center_y: 150,
        track_id: null,
        source: "dev-model",
        model_name: "yolov8n",
      },
    ];
    mockCameraApis(
      {
        connected: true,
        stream_url: "http://10.0.0.5:8090/stream.mjpg",
        frame_width: 640,
        frame_height: 480,
        fps: 15,
      },
      { frame_width: 640, frame_height: 480, timestamp: 1700000000, detections },
    );
    const renderSpy = vi.spyOn(overlayRenderModule, "renderDetectionOverlay").mockImplementation(() => {});

    render(<CameraPanel />);
    const img = await screen.findByRole("img");
    // jsdom never lays out real image content, so <img>.clientWidth/Height
    // are always 0 -- stub the actually-rendered display size CameraPanel
    // reads off the ref, matching what a real loaded <img> would report.
    Object.defineProperty(img, "clientWidth", { value: 320, configurable: true });
    Object.defineProperty(img, "clientHeight", { value: 240, configurable: true });

    fireEvent.load(img); // triggers the "stream reachable" state change the overlay effect depends on

    await waitFor(() => expect(renderSpy).toHaveBeenCalled());
    const lastCall = renderSpy.mock.calls[renderSpy.mock.calls.length - 1];
    const [, calledDetections, frameWidth, frameHeight, displayWidth, displayHeight] = lastCall;
    expect(calledDetections).toEqual(detections);
    expect(frameWidth).toBe(640);
    expect(frameHeight).toBe(480);
    expect(displayWidth).toBe(320);
    expect(displayHeight).toBe(240);
  });
});
