import { useEffect, useState } from "react";
import { getHealth } from "../api";
import type { HealthResponse, RosStatus } from "../types";

// Human-readable labels for GET /health's ros_status -- see
// gcs/backend/app/schemas.py's HealthResponse docstring and
// src/types.ts's RosStatus for what each value means. Reusing this same
// /health poll (already fetched here for the rosbridge target string)
// rather than adding a second, unrelated status mechanism -- see
// ConnectionPanel.tsx for the separate, telemetry-driven GCS<->Jetson/
// Jetson<->Pixhawk link indicators this is not a replacement for.
const ROS_STATUS_LABEL: Record<RosStatus, string> = {
  connected: "connected",
  disabled: "ROS disabled (GCS_ROS_ENABLED=false) -- local development mode",
  unavailable: "ROS unavailable -- rosbridge unreachable",
};

// Shows the literal configured rosbridge target so a developer can judge
// sim vs. real hardware for themselves -- the architecture deliberately
// keeps sim and real indistinguishable to backend logic (see
// custom-gcs/docs/DECISIONS.md D-0's hardware-isolation seam note), so
// this must not guess or label it "MOCK"/"REAL".
export default function Footer() {
  const [source, setSource] = useState<HealthResponse["telemetry_source"] | null>(null);
  const [target, setTarget] = useState<string | null>(null);
  const [rosStatus, setRosStatus] = useState<RosStatus | null>(null);

  useEffect(() => {
    let mounted = true;
    getHealth()
      .then((h) => {
        if (!mounted) return;
        setSource(h.telemetry_source ?? "rosbridge");
        setTarget(`${h.rosbridge_host}:${h.rosbridge_port}`);
        setRosStatus(h.ros_status);
      })
      .catch(() => {
        if (!mounted) return;
        setSource(null);
        setTarget(null);
        setRosStatus(null);
      });
    return () => {
      mounted = false;
    };
  }, []);

  if (source === "radio") {
    // No ROS link on the GCS at all: everything comes over the command radio.
    return <footer className="mt-4 text-dim text-[11px]">telemetry: MicroLR900 command radio (no Wi-Fi link)</footer>;
  }

  return (
    <footer className="mt-4 text-dim text-[11px]">
      rosbridge target: {target ?? "unknown"}
      {" — "}
      <span className={rosStatus && rosStatus !== "connected" ? "text-warn" : undefined}>
        ROS: {rosStatus ? ROS_STATUS_LABEL[rosStatus] : "unknown"}
      </span>
    </footer>
  );
}
