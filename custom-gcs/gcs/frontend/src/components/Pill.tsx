type PillKind = "ok" | "bad" | "warn" | "unknown";

const KIND_CLASSES: Record<PillKind, string> = {
  ok: "bg-ok/15 text-ok",
  bad: "bg-bad/15 text-bad",
  warn: "bg-warn/15 text-warn",
  unknown: "bg-dim/15 text-dim",
};

export default function Pill({ kind, children }: { kind: PillKind; children: string }) {
  return (
    <span className={`inline-block px-2 py-0.5 rounded-full text-xs font-semibold ${KIND_CLASSES[kind]}`}>
      {children}
    </span>
  );
}

// null/undefined -> UNKNOWN, matching the old prototype's fmtBool/fmtConn.
export function BoolPill({
  value,
  okLabel = "YES",
  badLabel = "NO",
}: {
  value: boolean | null | undefined;
  okLabel?: string;
  badLabel?: string;
}) {
  if (value === null || value === undefined) return <Pill kind="unknown">UNKNOWN</Pill>;
  return value ? <Pill kind="ok">{okLabel}</Pill> : <Pill kind="bad">{badLabel}</Pill>;
}

export function ConnPill({ value }: { value: boolean | null | undefined }) {
  if (value === null || value === undefined) return <Pill kind="unknown">UNKNOWN</Pill>;
  return value ? <Pill kind="ok">CONNECTED</Pill> : <Pill kind="bad">DISCONNECTED</Pill>;
}
