import { useEffect, useRef, useState } from "react";
import { getTelemetry } from "../api";
import type { TelemetryResponse } from "../types";

const POLL_INTERVAL_MS = 1000;

export interface UseTelemetryResult {
  data: TelemetryResponse | null;
  error: string | null;
  lastUpdatedAt: Date | null;
}

// Polls /api/telemetry at 1 Hz (matching the pre-React prototype's
// cadence). On a failed poll, the last good `data` is kept on screen
// (never blanked) while `error` is surfaced so the UI can show a
// "telemetry fetch failed" state instead of silently going stale.
export function useTelemetry(): UseTelemetryResult {
  const [data, setData] = useState<TelemetryResponse | null>(null);
  const [error, setError] = useState<string | null>(null);
  const [lastUpdatedAt, setLastUpdatedAt] = useState<Date | null>(null);
  const mountedRef = useRef(true);

  useEffect(() => {
    mountedRef.current = true;

    async function poll() {
      try {
        const t = await getTelemetry();
        if (!mountedRef.current) return;
        setData(t);
        setError(null);
        setLastUpdatedAt(new Date());
      } catch (e) {
        if (!mountedRef.current) return;
        setError(e instanceof Error ? e.message : String(e));
      }
    }

    poll();
    const id = setInterval(poll, POLL_INTERVAL_MS);
    return () => {
      mountedRef.current = false;
      clearInterval(id);
    };
  }, []);

  return { data, error, lastUpdatedAt };
}
