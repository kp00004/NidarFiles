import { useEffect, useState } from "react";
import Panel from "./Panel";
import { getPerceptionDetections, getPerceptionStatus } from "../api";
import type { Detection, PerceptionDetectionsResponse, PerceptionStatusResponse } from "../types";

const POLL_INTERVAL_MS = 1000;

function formatConfidence(confidence: number | null): string {
  return confidence != null ? `${Math.round(confidence * 100)}%` : "unavailable";
}

function formatBbox(bbox: Detection["bbox"]): string {
  const { x_min, y_min, x_max, y_max } = bbox;
  if (x_min == null || y_min == null || x_max == null || y_max == null) return "unavailable";
  return `(${x_min}, ${y_min}) – (${x_max}, ${y_max})`;
}

function formatTimestamp(timestamp: number | null): string {
  return timestamp != null ? new Date(timestamp * 1000).toLocaleTimeString() : "unavailable";
}

// Raw, image-space, UNCONFIRMED "person detected" output from a
// development-only pretrained detector running on the Jetson --
// deliberately distinct from, and never blended with, SurvivorsPanel's
// confirmed/localized survivor tags. See custom-gcs/CLAUDE.md and
// types.ts's Detection/SurvivorResponse comment.
export default function PerceptionPanel() {
  const [status, setStatus] = useState<PerceptionStatusResponse | null>(null);
  const [detections, setDetections] = useState<PerceptionDetectionsResponse | null>(null);
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    let mounted = true;

    async function poll() {
      try {
        const [s, d] = await Promise.all([getPerceptionStatus(), getPerceptionDetections()]);
        if (mounted) {
          setStatus(s);
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

  const cameraUnavailable = status?.camera_connected !== true;
  const detectorUnavailable = !cameraUnavailable && status?.detector_ready !== true;
  const detecting = !cameraUnavailable && !detectorUnavailable && status?.person_count == null;
  const people = detections?.detections ?? [];

  return (
    <Panel title="Perception (dev model — person detection, not confirmed survivors)">
      {error && <div className="text-bad text-xs mb-2">perception fetch failed: {error}</div>}
      {cameraUnavailable ? (
        <div className="text-dim text-xs">CAMERA UNAVAILABLE</div>
      ) : detectorUnavailable ? (
        <div className="text-dim text-xs">
          PERCEPTION UNAVAILABLE
          {status?.detector_enabled === false && " — detector disabled"}
          {status?.detector_enabled === true && " — detector not ready yet"}
        </div>
      ) : detecting ? (
        <div className="text-dim text-xs">DETECTING...</div>
      ) : people.length === 0 ? (
        <div className="text-dim text-xs">No detection</div>
      ) : (
        <div>
          <div className="text-xs text-warn font-semibold mb-2">
            {people.length === 1 ? "Person detected" : `Multiple people (${people.length})`}
          </div>
          <div className="text-xs space-y-1">
            {people.map((p, i) => (
              <div
                key={p.detection_id ?? i}
                className="flex justify-between border-b border-b-[#1d252d] last:border-b-0 py-0.5"
              >
                <span className="text-dim">{p.detection_id ?? "?"}</span>
                <span className="tabular-nums">
                  {formatConfidence(p.confidence)} {formatBbox(p.bbox)} @ {formatTimestamp(detections?.timestamp ?? null)}
                </span>
              </div>
            ))}
          </div>
        </div>
      )}
    </Panel>
  );
}
