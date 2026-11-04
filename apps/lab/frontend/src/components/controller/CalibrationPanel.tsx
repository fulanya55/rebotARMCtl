/** 1.2 Calibration: independent steps, per joint verification, versioned per arm serial. */
import React, { useCallback, useEffect, useState } from "react";
import { useTranslation } from "react-i18next";
import { Button } from "@/components/ui/button";
import { Input } from "@/components/ui/input";
import { Badge } from "@/components/ui/badge";
import { toast } from "@/components/ui/sonner";
import { Select, SelectContent, SelectItem, SelectTrigger, SelectValue } from "@/components/ui/select";
import { ArmName, JOINTS, quietly, useArmSample, useController } from "@/contexts/ControllerContext";
import { ArmSelect, EmptyState, Field, JointTable, Section } from "./common";

interface Draft {
  serial: string;
  model: string;
  version: number;
  zero: number[];
  direction: number[];
  lower: number[];
  upper: number[];
  gripper_closed: number;
  gripper_open: number;
  steps_done: string[];
  note: string;
}

interface CalState {
  draft: Draft;
  applied_version: number | null;
  versions: number[];
  tracking_range: boolean;
}

const CalibrationPanel: React.FC = () => {
  const { t } = useTranslation();
  const { status, api, modelOf } = useController();
  const [arm, setArm] = useState<ArmName>("left_follower");
  const [state, setState] = useState<CalState | null>(null);
  const [note, setNote] = useState("");
  const [verifying, setVerifying] = useState<number | null>(null);
  const armStatus = status?.arms.find((a) => a.arm === arm);
  const bound = !!armStatus?.bound;
  const sample = useArmSample(arm);
  const spec = modelOf(arm);

  const refresh = useCallback(() => {
    if (!bound) {
      setState(null);
      return;
    }
    api<CalState>(`/calibration/${arm}`, undefined, { quiet: true }).then(setState).catch(() => setState(null));
  }, [api, arm, bound]);

  useEffect(refresh, [refresh]);

  const run = (path: string, body: Record<string, unknown>) =>
    quietly(api<CalState>(path, { arm, ...body }).then((next) => setState(next)));

  const verify = async (joint: number) => {
    setVerifying(joint);
    try {
      const result = await api<{ direction_ok: boolean; tracking_ok: boolean; moved: number; commanded: number; error: string | null }>("/calibration/verify", {
        arm,
        joint,
        amplitude: 0.1,
      });
      const summary = t("controller.calibration.verifyResult", {
        joint: JOINTS[joint],
        commanded: result.commanded.toFixed(3),
        moved: result.moved.toFixed(3),
      });
      if (result.direction_ok && result.tracking_ok) toast.success(summary);
      else toast.error(`${summary} ${t("controller.calibration.verifyFailed")}`);
    } catch {
      /* shown by api */
    } finally {
      setVerifying(null);
    }
  };

  if (!status?.connected) return <EmptyState>{t("controller.common.connectFirst")}</EmptyState>;

  const done = (step: string) => state?.draft.steps_done.includes(step);
  const energized = armStatus?.torque_on;

  return (
    <div className="grid gap-4 xl:grid-cols-[1fr_320px]">
      <div className="space-y-4">
        <Section title={t("controller.calibration.armTitle")} hint={t("controller.calibration.armHint")}>
          <div className="flex flex-wrap items-end gap-3">
            <Field label={t("controller.common.arm")}>
              <ArmSelect value={arm} onChange={setArm} />
            </Field>
            {state && (
              <>
                <Field label={t("controller.device.serial")}>
                  <span className="pb-1.5 text-sm font-medium text-foreground">{state.draft.serial}</span>
                </Field>
                <Field label={t("controller.calibration.applied")}>
                  <Badge variant="secondary">{state.applied_version ? `v${state.applied_version}` : t("controller.calibration.defaults")}</Badge>
                </Field>
                <Field label={t("controller.calibration.versions")}>
                  <Select onValueChange={(v) => run("/calibration/load", { version: Number(v) })} disabled={!state.versions.length}>
                    <SelectTrigger className="h-8 w-28">
                      <SelectValue placeholder={t("controller.calibration.load")} />
                    </SelectTrigger>
                    <SelectContent>
                      {state.versions.map((v) => (
                        <SelectItem key={v} value={String(v)}>
                          v{v}
                        </SelectItem>
                      ))}
                    </SelectContent>
                  </Select>
                </Field>
              </>
            )}
          </div>
          {energized && <p className="mt-3 text-xs text-destructive">{t("controller.calibration.energized")}</p>}
        </Section>

        {state && (
          <>
            <Section
              title={t("controller.calibration.zeroTitle")}
              hint={t("controller.calibration.zeroHint")}
              actions={
                <Button size="sm" variant="outline" disabled={!!energized} onClick={() => run("/calibration/capture-zero", {})} data-testid="capture-zero">
                  {t("controller.calibration.captureAll")} {done("zero") && "✓"}
                </Button>
              }
            >
              <div className="overflow-x-auto">
                <table className="w-full min-w-[620px] text-sm">
                  <thead className="text-xs text-muted-foreground">
                    <tr>
                      <th className="py-1 text-left font-normal">{t("controller.common.joint")}</th>
                      <th className="py-1 text-right font-normal">{t("controller.common.measured")}</th>
                      <th className="py-1 text-right font-normal">{t("controller.calibration.zero")}</th>
                      <th className="py-1 text-center font-normal">{t("controller.calibration.direction")}</th>
                      <th className="py-1 text-right font-normal">{t("controller.calibration.range")}</th>
                      <th />
                    </tr>
                  </thead>
                  <tbody>
                    {JOINTS.slice(0, 6).map((name, joint) => (
                      <tr key={name} className="border-t tabular-nums">
                        <td className="py-1.5">{name}</td>
                        <td className="py-1.5 text-right">{sample ? sample.position[joint].toFixed(3) : "-"}</td>
                        <td className="py-1.5 text-right">{state.draft.zero[joint].toFixed(3)}</td>
                        <td className="py-1.5 text-center">
                          <Button
                            size="sm"
                            variant="ghost"
                            className="h-6 px-2"
                            disabled={!!energized}
                            onClick={() => run("/calibration/direction", { joint, direction: state.draft.direction[joint] > 0 ? -1 : 1 })}
                          >
                            {state.draft.direction[joint] > 0 ? "+1" : "-1"}
                          </Button>
                        </td>
                        <td className="py-1.5 text-right text-xs">
                          {state.draft.lower[joint].toFixed(2)} … {state.draft.upper[joint].toFixed(2)}
                        </td>
                        <td className="py-1.5 pl-2 text-right">
                          <Button size="sm" variant="ghost" className="h-6 px-2" disabled={!!energized} onClick={() => run("/calibration/capture-zero", { joints: [joint] })}>
                            {t("controller.calibration.zeroJoint")}
                          </Button>
                          <Button
                            size="sm"
                            variant="ghost"
                            className="h-6 px-2"
                            disabled={!spec?.can_execute || verifying !== null || status.source !== "none"}
                            title={spec?.can_execute ? undefined : t("controller.calibration.verifyByHand")}
                            onClick={() => void verify(joint)}
                          >
                            {verifying === joint ? "…" : t("controller.calibration.verify")}
                          </Button>
                        </td>
                      </tr>
                    ))}
                  </tbody>
                </table>
              </div>
            </Section>

            <div className="grid gap-4 md:grid-cols-2">
              <Section
                title={t("controller.calibration.rangeTitle")}
                hint={t("controller.calibration.rangeHint")}
                actions={
                  state.tracking_range ? (
                    <Button size="sm" onClick={() => run("/calibration/range-stop", {})}>
                      {t("controller.calibration.rangeStop")}
                    </Button>
                  ) : (
                    <Button size="sm" variant="outline" disabled={!!energized} onClick={() => run("/calibration/range-start", {})}>
                      {t("controller.calibration.rangeStart")} {done("range") && "✓"}
                    </Button>
                  )
                }
              >
                {state.tracking_range && <p className="text-xs text-info">{t("controller.calibration.rangeRecording")}</p>}
              </Section>
              <Section title={t("controller.calibration.gripperTitle")} hint={t("controller.calibration.gripperHint")}>
                <div className="flex flex-wrap items-center gap-2">
                  <Button size="sm" variant="outline" disabled={!!energized} onClick={() => run("/calibration/gripper", { which: "closed" })}>
                    {t("controller.calibration.gripperClosed")}
                  </Button>
                  <Button size="sm" variant="outline" disabled={!!energized} onClick={() => run("/calibration/gripper", { which: "open" })}>
                    {t("controller.calibration.gripperOpen")}
                  </Button>
                  <span className="text-xs tabular-nums text-muted-foreground">
                    {state.draft.gripper_closed.toFixed(3)} … {state.draft.gripper_open.toFixed(3)} rad
                  </span>
                </div>
              </Section>
            </div>

            <Section title={t("controller.calibration.saveTitle")} hint={t("controller.calibration.saveHint")}>
              <div className="flex flex-wrap items-end gap-2">
                <Field label={t("controller.calibration.note")} className="min-w-64 flex-1">
                  <Input className="h-8" value={note} onChange={(e) => setNote(e.target.value)} />
                </Field>
                <Button size="sm" disabled={!!energized} onClick={() => run("/calibration/save", { note })} data-testid="save-calibration">
                  {t("controller.calibration.save")}
                </Button>
              </div>
            </Section>
          </>
        )}
      </div>
      <Section title={t("controller.common.liveState")}>
        <JointTable arm={arm} />
      </Section>
    </div>
  );
};

export default CalibrationPanel;
