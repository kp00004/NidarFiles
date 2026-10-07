import type { ReactNode } from "react";

interface PanelProps {
  title: string;
  children: ReactNode;
  className?: string;
}

export default function Panel({ title, children, className = "" }: PanelProps) {
  return (
    <div className={`bg-panel border border-border rounded-lg p-3 ${className}`}>
      <h2 className="text-xs uppercase tracking-wide text-dim mb-2.5">{title}</h2>
      {children}
    </div>
  );
}

interface RowProps {
  label: string;
  children: ReactNode;
}

export function Row({ label, children }: RowProps) {
  return (
    <div className="flex justify-between py-1 border-b border-b-[#1d252d] last:border-b-0 text-[13px]">
      <span className="text-dim">{label}</span>
      <span className="tabular-nums">{children}</span>
    </div>
  );
}
