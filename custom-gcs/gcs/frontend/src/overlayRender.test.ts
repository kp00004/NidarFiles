import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import { renderDetectionOverlay } from "./overlayRender";
import type { Detection } from "./types";

// jsdom doesn't implement a real 2D canvas context (getContext("2d")
// returns null unless the optional "canvas" npm package is installed --
// see node_modules/jsdom's not-implemented stub), and this repo doesn't
// add new dependencies for tests. Stub `getContext` with a fake context
// object exposing the handful of drawing methods overlayRender.ts
// actually calls, so behavior (call counts/args) is still verifiable
// without pulling in a real canvas renderer.
function makeFakeContext() {
  return {
    clearRect: vi.fn(),
    strokeRect: vi.fn(),
    fillRect: vi.fn(),
    fillText: vi.fn(),
    measureText: vi.fn(() => ({ width: 40 })),
    strokeStyle: "",
    fillStyle: "",
    lineWidth: 0,
    font: "",
  };
}

function makeCanvas(width = 200, height = 150): { canvas: HTMLCanvasElement; ctx: ReturnType<typeof makeFakeContext> } {
  const canvas = document.createElement("canvas");
  canvas.width = width;
  canvas.height = height;
  const ctx = makeFakeContext();
  vi.spyOn(canvas, "getContext").mockReturnValue(ctx as unknown as CanvasRenderingContext2D);
  return { canvas, ctx };
}

function makeDetection(overrides: Partial<Detection> = {}): Detection {
  return {
    detection_id: "det-1",
    class_name: "person",
    confidence: 0.87,
    bbox: { x_min: 10, y_min: 20, x_max: 50, y_max: 100 },
    center_x: 30,
    center_y: 60,
    track_id: null,
    source: "dev-model",
    model_name: "yolov8n",
    ...overrides,
  };
}

beforeEach(() => {
  vi.restoreAllMocks();
});

afterEach(() => {
  vi.restoreAllMocks();
});

describe("renderDetectionOverlay", () => {
  it("does not throw for an empty detections array", () => {
    const { canvas } = makeCanvas();
    expect(() => renderDetectionOverlay(canvas, [], 640, 480, 200, 150)).not.toThrow();
  });

  it("does not throw and skips detections with any null bbox field", () => {
    const { canvas, ctx } = makeCanvas();
    const detections = [
      makeDetection({ bbox: { x_min: null, y_min: 20, x_max: 50, y_max: 100 } }),
      makeDetection({ bbox: { x_min: 10, y_min: null, x_max: 50, y_max: 100 } }),
      makeDetection({ bbox: { x_min: 10, y_min: 20, x_max: null, y_max: 100 } }),
      makeDetection({ bbox: { x_min: 10, y_min: 20, x_max: 50, y_max: null } }),
    ];

    expect(() => renderDetectionOverlay(canvas, detections, 640, 480, 200, 150)).not.toThrow();
    expect(ctx.strokeRect).not.toHaveBeenCalled();
  });

  it("draws one stroked box and one filled label per valid detection", () => {
    const { canvas, ctx } = makeCanvas();
    const detections = [makeDetection(), makeDetection({ detection_id: "det-2", confidence: 0.5 })];

    renderDetectionOverlay(canvas, detections, 640, 480, 200, 150);

    expect(ctx.strokeRect).toHaveBeenCalledTimes(2);
    expect(ctx.fillRect).toHaveBeenCalledTimes(2);
    expect(ctx.fillText).toHaveBeenCalledTimes(2);
    expect(ctx.fillText.mock.calls[0][0]).toBe("person 87%");
    expect(ctx.fillText.mock.calls[1][0]).toBe("person 50%");
  });

  it("does not throw when the frame size is unknown (zero)", () => {
    const { canvas, ctx } = makeCanvas();
    expect(() => renderDetectionOverlay(canvas, [makeDetection()], 0, 0, 200, 150)).not.toThrow();
    expect(ctx.strokeRect).not.toHaveBeenCalled();
  });

  it("scales bbox coordinates from frame space to display space exactly", () => {
    const { canvas, ctx } = makeCanvas();
    const detection = makeDetection({ bbox: { x_min: 100, y_min: 50, x_max: 300, y_max: 150 } });

    // frame 400x200 -> display 200x100 => scaleX = scaleY = 0.5
    renderDetectionOverlay(canvas, [detection], 400, 200, 200, 100);

    expect(ctx.strokeRect).toHaveBeenCalledWith(50, 25, 100, 50);
  });

  it("scales x and y independently when the frame and display aspect ratios differ", () => {
    const { canvas, ctx } = makeCanvas();
    const detection = makeDetection({ bbox: { x_min: 40, y_min: 60, x_max: 80, y_max: 90 } });

    // frame 400x300 -> display 100x150 => scaleX = 0.25, scaleY = 0.5
    renderDetectionOverlay(canvas, [detection], 400, 300, 100, 150);

    // x: 40*0.25=10, y: 60*0.5=30, w: (80-40)*0.25=10, h: (90-60)*0.5=15
    expect(ctx.strokeRect).toHaveBeenCalledWith(10, 30, 10, 15);
  });

  it("draws each of multiple detections at its own correctly scaled position", () => {
    const { canvas, ctx } = makeCanvas();
    const detections = [
      makeDetection({ detection_id: "a", bbox: { x_min: 0, y_min: 0, x_max: 100, y_max: 100 } }),
      makeDetection({ detection_id: "b", bbox: { x_min: 200, y_min: 100, x_max: 300, y_max: 200 } }),
    ];

    // frame 400x400 -> display 200x200 => scale 0.5
    renderDetectionOverlay(canvas, detections, 400, 400, 200, 200);

    expect(ctx.strokeRect).toHaveBeenCalledTimes(2);
    expect(ctx.strokeRect).toHaveBeenNthCalledWith(1, 0, 0, 50, 50);
    expect(ctx.strokeRect).toHaveBeenNthCalledWith(2, 100, 50, 50, 50);
  });

  it("clears the canvas using its own pixel dimensions, not the frame/display sizes", () => {
    const { canvas, ctx } = makeCanvas(200, 150);
    renderDetectionOverlay(canvas, [], 640, 480, 200, 150);
    expect(ctx.clearRect).toHaveBeenCalledWith(0, 0, 200, 150);
  });
});
