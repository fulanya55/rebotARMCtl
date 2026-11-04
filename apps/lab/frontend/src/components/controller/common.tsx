import React from "react";
import { useTranslation } from "react-i18next";
import { Select, SelectContent, SelectItem, SelectTrigger, SelectValue } from "@/components/ui/select";
import { ArmName, ARMS, JOINTS, useArmSample, useController } from "@/contexts/ControllerContext";

export const Section: React.FC<{ title: string; hint?: string; actions?: React.ReactNode; children: React.ReactNode }> = ({
  title,
  hint,
  actions,
  children,
}) => (
  <section className="rounded-lg border bg-card p-4">
    <div className="mb-3 flex flex-wrap items-start justify-between gap-2">
      <div>
        <h3 className="text-sm font-semibold">{title}</h3>
        {hint && <p className="mt-0.5 max-w-prose text-xs text-muted-foreground">{hint}</p>}
      </div>
      {actions && <div className="flex flex-wrap gap-2">{actions}</div>}
    </div>
    {children}
  </section>
);

export const Field: React.FC<{ label: string; children: React.ReactNode; className?: string }> = ({ label, children, className }) => (
  <label className={`flex flex-col gap-1 text-xs text-muted-foreground ${className ?? ""}`}>
    <span>{label}</span>
    {children}
  </label>
);

/** Choose among bound arms. An unbound role is listed disabled, so the reason is visible. */
export const ArmSelect: React.FC<{
  value: string;
  onChange: (arm: ArmName) => void;
  filter?: (arm: ArmName) => boolean;
}> = ({ value, onChange, filter }) => {
  const { t } = useTranslation();
  const { status } = useController();
  const arms = ARMS.filter((arm) => (filter ? filter(arm) : true));
  return (
    <Select value={value} onValueChange={(next) => onChange(next as ArmName)}>
      <SelectTrigger className="h-8 w-44">
        <SelectValue />
      </SelectTrigger>
      <SelectContent>
        {arms.map((arm) => {
          const bound = status?.arms.find((a) => a.arm === arm)?.bound;
          return (
            <SelectItem key={arm} value={arm} disabled={!bound}>
              {t(`controller.arm.${arm}` as never)}
              {!bound && ` (${t("controller.arm.unbound")})`}
            </SelectItem>
          );
        })}
      </SelectContent>
    </Select>
  );
};

/** Live joint table: measured against target, with stale joints marked. */
export const JointTable: React.FC<{ arm: string; compact?: boolean }> = ({ arm, compact }) => {
  const { t } = useTranslation();
  const sample = useArmSample(arm);
  if (!sample) return <p className="text-xs text-muted-foreground">{t("controller.common.noFeedback")}</p>;
  return (
    <table className="w-full text-xs tabular-nums">
      <thead className="text-muted-foreground">
        <tr>
          <th className="py-1 text-left font-normal">{t("controller.common.joint")}</th>
          <th className="py-1 text-right font-normal">{t("controller.common.measured")}</th>
          <th className="py-1 text-right font-normal">{t("controller.common.target")}</th>
          {!compact && <th className="py-1 text-right font-normal">{t("controller.common.torque")}</th>}
          {!compact && <th className="py-1 text-right font-normal">°C</th>}
        </tr>
      </thead>
      <tbody>
        {JOINTS.map((name, index) => {
          const valid = !!(sample.valid_mask & (1 << index));
          return (
            <tr key={name} className={valid ? "" : "text-destructive"}>
              <td className="py-0.5">{name}</td>
              <td className="py-0.5 text-right">{sample.position[index].toFixed(3)}</td>
              <td className="py-0.5 text-right">{sample.flags & 8 ? sample.target[index].toFixed(3) : "-"}</td>
              {!compact && <td className="py-0.5 text-right">{sample.torque[index].toFixed(2)}</td>}
              {!compact && <td className="py-0.5 text-right">{sample.temperature[index]}</td>}
            </tr>
          );
        })}
      </tbody>
    </table>
  );
};

export const EmptyState: React.FC<{ children: React.ReactNode }> = ({ children }) => (
  <div className="rounded-lg border border-dashed p-8 text-center text-sm text-muted-foreground">{children}</div>
);

export const Stat: React.FC<{ label: string; value: React.ReactNode }> = ({ label, value }) => (
  <div>
    <div className="text-xs text-muted-foreground">{label}</div>
    <div className="text-sm font-medium tabular-nums">{value}</div>
  </div>
);
