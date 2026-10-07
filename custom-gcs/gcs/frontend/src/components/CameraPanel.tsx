import { useEffect, useRef, useState } from "react";
import Panel from "./Panel";
import { getCameraStatus, getPerceptionDetections } from "../api";
import { renderDetectionOverlay } from "../overlayRender";
import type { CameraStatusResponse, PerceptionDetectionsResponse } from "../types";

const POLL_INTERVAL_MS = 1000;

// Live video is a direct browser->Jetson MJPEG-over-HTTP stream, NOT
// proxied through the FastAPI backend or rosbridge (see
// docs/DECISIONS.md D-6) -- this panel only asks the backend for
// `stream_url` and then points an <img> straight at the Jetson. That
// means the browser's own ability to reach the stream is a genuinely
// separate failure mode from what the backend/ROS side reports via
// `camera.connected` (network segmentation between the GCS laptop and
// the Jetson is possible even when the backend's own link is fine) --
// tracked here as its own `streamReachable` state, never conflated with
// `camera.connected`.
export default function CameraPanel() {
  const [camera, setCamera] = useState<CameraStatusResponse | null>(null);
  const [detections, setDetections] = useState<PerceptionDetectionsResponse | null>(null);
  const [error, setError] = useState<string | null>(null);
  const [streamReachable, setStreamReachable] = useState<boolean | null>(null);
  const imgRef = useRef<HTMLImageElement | null>(null);
  const canvasRef = useRef<HTMLCanvasElement | null>(null);

  useEffect(() => {
    let mounted = true;

    async function poll() {
      try {
        const [c, d] = await Promise.all([getCameraStatus(), getPerceptionDetections()]);
        if (mounted) {
          setCamera(c);
          setDetections(d);
          setError(null);
        }
      } catch (e) {
        if (mounted) setError(e instanceof Error ? e.message : String(e));
      }
    }

    poll();
    const id = setInterval(poll, POLL_INTERVAL_MS);
    return () => {
      mounted = false;
      clearInterval(id);
    };
  }, []);

  const hasStream = camera?.connected === true && !!camera.stream_url;

  // Re-draw the overlay whenever detections change, whenever the <img>
  // finishes loading a frame (its clientWidth/clientHeight are only
  // meaningful once it has actual image content), and on stream-url
  // changes. The ref may not be mounted yet (or have zero size, e.g.
  // still loading) -- skip rendering rather than throw.
  useEffect(() => {
    const canvas = canvasRef.current;
    const img = imgRef.current;
    if (!canvas || !img || !hasStream) return;
    const displayWidth = img.clientWidth;
    const displayHeight = img.clientHeight;
    if (!displayWidth || !displayHeight) return;

    canvas.width = displayWidth;
    canvas.height = displayHeight;
    canvas.style.width = `${displayWidth}px`;
    canvas.style.height = `${displayHeight}px`;

    renderDetectionOverlay(
      canvas,
      detections?.detections ?? [],
      detections?.frame_width ?? 0,
      detections?.frame_height ?? 0,
      displayWidth,
      displayHeight,
    );
  }, [detections, hasStream, streamReachable]);

  return (
    <Panel title="Live Camera" className="col-span-full">
      {error && <div className="text-bad text-xs mb-2">camera fetch failed: {error}</div>}
      {camera?.connected !== true ? (
        <div className="text-dim text-xs">CAMERA DISCONNECTED</div>
      ) : !camera.stream_url ? (
        <div className="text-dim text-xs">CAMERA CONNECTED — NO VIDEO STREAM</div>
      ) : (
        <div>
          <div className="flex items-center justify-between mb-2 text-xs">
            <span className={streamReachable === false ? "text-bad font-semibold" : "text-ok font-semibold"}>
              {streamReachable === false ? "● OFFLINE" : "● LIVE"}
            </span>
            <span className="text-dim">
              {camera.frame_width != null && camera.frame_height != null
                ? `${camera.frame_width} × ${camera.frame_height}`
                : "unavailable"}
              {" · "}
              {camera.fps != null ? `${camera.fps} fps` : "unavailable"}
            </span>
          </div>
          <div style={{ position: "relative" }}>
            <img
              ref={imgRef}
              src={camera.stream_url}
              onLoad={() => setStreamReachable(true)}
              onError={() => setStreamReachable(false)}
              style={{ width: "100%", display: "block" }}
              alt="live drone camera feed"
            />
            <canvas
              ref={canvasRef}
              data-testid="detection-overlay-canvas"
              style={{ position: "absolute", top: 0, left: 0, pointerEvents: "none" }}
            />
          </div>
        </div>
      )}
    </Panel>
  );
}
