/** 1.4 Visual teaching: sliders, joint drag, end-effector drag, teach points, hand guidance. */
import React, { useCallback, useEffect, useMemo, useRef, useState } from "react";
import { useTranslation } from "react-i18next";
import { Button } from "@/components/ui/button";
import { Input } from "@/components/ui/input";
import { Switch } from "@/components/ui/switch";
import { toast } from "@/components/ui/sonner";
import { Select, SelectContent, SelectItem, SelectTrigger, SelectValue } from "@/components/ui/select";
import { ArrowDown, ArrowUp, Trash2 } from "lucide-react";
import { ArmName, JOINTS, quietly, useController } from "@/contexts/ControllerContext";
import { ArmSelect, EmptyState, Field, Section } from "./common";
import RobotView3D from "./RobotView3D";

interface TeachPoint {
  name: string;
  q: number[];
  move_time_s: number;
  dwell_s: number;
}

const TARGET_PERIOD_MS = 33;

const TeachPanel: React.FC = () => {
  const { t } = useTranslation();
  const { status, api, samplesRef, modelOf } = useController();
  const [arm, setArm] = useState<ArmName>("left_follower");
  const spec = modelOf(arm);
  const [target, setTarget] = useState<number[] | null>(null);
  const [live, setLive] = useState(false);
  const [points, setPoints] = useState<TeachPoint[]>([]);
  const [programName, setProgramName] = useState("program_1");
  const [programs, setPrograms] = useState<{ name: string; points: number }[]>([]);
  const [previewing, setPreviewing] = useState(false);
  const [ikNote, setIkNote] = useState<string | null>(null);
  const previewTimer = useRef<ReturnType<typeof setInterval> | null>(null);
  const lastSend = useRef(0);
  const pending = useRef<number[] | null>(null);
  const ikInFlight = useRef(false);

  const armStatus = status?.arms.find((a) => a.arm === arm);
  const canExecute = !!spec?.can_execute;
  const running = status?.source && status.source !== "none" && status.source !== "teach";
  const guided = !!status?.guide[arm];

  const refreshPrograms = useCallback(() => {
    api<{ name: string; points: number }[]>("/programs", undefined, { quiet: true }).then(setPrograms).catch(() => undefined);
  }, [api]);
  useEffect(refreshPrograms, [refreshPrograms]);

  useEffect(() => {
    setTarget(null);
    setLive(false);
  }, [arm]);

  const currentPose = useCallback(() => samplesRef.current[arm]?.position ?? null, [arm, samplesRef]);

  // Latest wins, at a bounded rate, so a fast drag cannot queue up motion behind it.
  const sendTarget = useCallback(
    (pose: number[]) => {
      pending.current = pose;
      const now = performance.now();
      if (now - lastSend.current < TARGET_PERIOD_MS) return;
      lastSend.current = now;
      quietly(api("/motion/target", { arm, pose, source: "teach" }, { quiet: true }));
    },
    [api, arm],
  );

  const updateTarget = useCallback(
    (pose: number[]) => {
      setTarget(pose);
      if (live) sendTarget(pose);
    },
    [live, sendTarget],
  );

  const flushDrag = useCallback(() => {
    if (live && pending.current) {
      quietly(api("/motion/target", { arm, pose: pending.current, source: "teach" }, { quiet: true }));
    }
  }, [api, arm, live]);

  const pose = useMemo(() => target ?? currentPose() ?? [0, 0, 0, 0, 0, 0, 0], [target, currentPose]);

  const onJointDrag = useCallback(
    (joint: string, value: number) => {
      const index = JOINTS.indexOf(joint as (typeof JOINTS)[number]);
      if (index < 0) return;
      const base = [...(target ?? currentPose() ?? [0, 0, 0, 0, 0, 0, 0])];
      base[index] = value;
      updateTarget(base);
    },
    [target, currentPose, updateTarget],
  );

  const onHandleDrag = useCallback(
    async (xyz: number[], rpy: number[]) => {
      if (!spec || ikInFlight.current) return;
      const seed = target ?? currentPose();
      if (!seed) return;
      ikInFlight.current = true;
      try {
        const result = await api<{ q: number[]; converged: boolean; near_singular: boolean; position_error: number }>(
          "/kinematics/ik",
          { model: spec.key, xyz, rpy, seed: seed.slice(0, 6) },
          { quiet: true },
        );
        const next = [...result.q, seed[6] ?? 0];
        updateTarget(next);
        setIkNote(!result.converged ? t("controller.teach.ikUnreachable", { error: (result.position_error * 1000).toFixed(1) }) : result.near_singular ? t("controller.teach.ikSingular") : null);
      } catch {
        setIkNote(t("controller.teach.ikFailed"));
      } finally {
        ikInFlight.current = false;
      }
    },
    [api, spec, target, currentPose, updateTarget, t],
  );

  const addPoint = () => {
    const q = target ?? currentPose();
    if (!q) {
      toast.error(t("controller.common.noFeedback"));
      return;
    }
    setPoints((list) => [...list, { name: `P${list.length + 1}`, q: [...q], move_time_s: 2.0, dwell_s: 0 }]);
  };

  /** A point from the leader arm's measured pose, mapped through the pair's mapping. */
  const addFromLeader = async () => {
    const side = arm.startsWith("left") ? "left" : "right";
    const leader = samplesRef.current[`${side}_leader`];
    if (!leader) {
      toast.error(t("controller.common.noFeedback"));
      return;
    }
    const config = await api<{ pairs: Record<string, { scale: number[]; offset: number[]; source_joint: number[] }> }>("/teleop/config");
    const pair = config.pairs[side === "left" ? "0" : "1"];
    const q = pair.scale.map((scale, j) => scale * leader.position[pair.source_joint[j]] + pair.offset[j]);
    setPoints((list) => [...list, { name: `P${list.length + 1}`, q, move_time_s: 2.0, dwell_s: 0 }]);
  };

  const editPoint = (index: number, change: Partial<TeachPoint>) => setPoints((list) => list.map((p, i) => (i === index ? { ...p, ...change } : p)));
  const movePoint = (index: number, delta: number) =>
    setPoints((list) => {
      const next = [...list];
      const [item] = next.splice(index, 1);
      next.splice(index + delta, 0, item);
      return next;
    });

  const program = () => ({ name: programName, arm, model: spec?.key ?? "", points });

  const stopPreview = useCallback(() => {
    if (previewTimer.current) clearInterval(previewTimer.current);
    previewTimer.current = null;
    setPreviewing(false);
  }, []);
  useEffect(() => stopPreview, [stopPreview]);

  const preview = async () => {
    stopPreview();
    const result = await api<{ dt_s: number; duration_s: number; points: number[][] }>("/programs/preview", {
      program: program(),
      start: currentPose(),
      dt_s: 0.02,
    });
    const sequence = result.points;
    if (!sequence.length) return;
    let index = 0;
    setPreviewing(true);
    previewTimer.current = setInterval(() => {
      setTarget(sequence[index]);
      index += 1;
      if (index >= sequence.length) stopPreview();
    }, result.dt_s * 1000);
  };

  const execute = () => {
    setLive(false);
    setTarget(null);
    quietly(api("/programs/execute", { program: program(), dt_s: 0.02 }));
  };

  const save = () => quietly(api("/programs", program()).then(refreshPrograms));
  const load = (name: string) =>
    quietly(
      api<{ name: string; points: TeachPoint[] }>(`/programs/${encodeURIComponent(name)}`).then((p) => {
        setProgramName(p.name);
        setPoints(p.points);
      }),
    );
  const remove = () => quietly(api(`/programs/${encodeURIComponent(programName)}`, undefined, { method: "DELETE" }).then(refreshPrograms));

  if (!status?.connected) return <EmptyState>{t("controller.common.connectFirst")}</EmptyState>;

  return (
    <div className="grid gap-4 xl:grid-cols-[minmax(0,1fr)_400px]">
      <div className="space-y-4">
        <Section
          title={t("controller.teach.viewTitle")}
          hint={t("controller.teach.viewHint")}
          actions={
            <div className="flex items-center gap-3">
              <ArmSelect value={arm} onChange={setArm} />
              <label className="flex items-center gap-1.5 text-xs">
                <Switch checked={live} disabled={!canExecute || !!running || guided} onCheckedChange={setLive} data-testid="live-target" />
                {t("controller.teach.live")}
              </label>
            </div>
          }
        >
          {spec ? (
            <RobotView3D
              arm={arm}
              urdf={spec.urdf}
              endEffector={spec.end_effector}
              target={target}
              interactive={!previewing && !guided && canExecute}
              onJointDrag={onJointDrag}
              onHandleDrag={(xyz, rpy) => void onHandleDrag(xyz, rpy)}
              onDragEnd={flushDrag}
            />
          ) : (
            <EmptyState>{t("controller.common.bindFirst")}</EmptyState>
          )}
          {ikNote && <p className="mt-2 text-xs text-warning">{ikNote}</p>}
          {!canExecute && spec && <p className="mt-2 text-xs text-muted-foreground">{t("controller.teach.readOnlyArm")}</p>}
        </Section>

        {spec?.can_guide && (
          <Section title={t("controller.teach.guideTitle")} hint={t("controller.teach.guideHint")}>
            <div className="flex flex-wrap gap-2">
              <Button size="sm" variant="outline" disabled={guided || !!running || status.source === "teach"} onClick={() => quietly(api("/guide/start", { arm, gravity: true }))}>
                {t("controller.teach.guideGravity")}
              </Button>
              <Button size="sm" variant="outline" disabled={guided || !!running || status.source === "teach"} onClick={() => quietly(api("/guide/start", { arm, gravity: false }))}>
                {t("controller.teach.guideLow")}
              </Button>
              <Button size="sm" variant="outline" disabled={!guided} onClick={() => quietly(api("/guide/stop", { arm }))}>
                {t("controller.teach.guideStop")}
              </Button>
            </div>
          </Section>
        )}
      </div>

      <div className="space-y-4">
        <Section
          title={t("controller.teach.jointsTitle")}
          actions={
            <>
              <Button size="sm" variant="ghost" onClick={() => setTarget(null)}>
                {t("controller.teach.reset")}
              </Button>
              <Button size="sm" variant="outline" disabled={!canExecute || !target || !!running || live} onClick={() => target && quietly(api("/motion/move-to", { arm, pose: target, duration_s: 2.0, source: "teach" }))}>
                {t("controller.teach.moveTo")}
              </Button>
            </>
          }
        >
          <div className="space-y-1.5">
            {JOINTS.map((name, index) => {
              const low = spec?.lower[index] ?? -3.14;
              const high = spec?.upper[index] ?? 3.14;
              return (
                <div key={name} className="grid grid-cols-[64px_1fr_56px] items-center gap-2 text-xs">
                  <span>{name}</span>
                  <input
                    type="range"
                    aria-label={name}
                    min={low}
                    max={high}
                    step={0.005}
                    value={pose[index] ?? 0}
                    disabled={!canExecute || previewing}
                    onChange={(e) => {
                      const next = [...pose];
                      next[index] = Number(e.target.value);
                      updateTarget(next);
                    }}
                    onPointerUp={flushDrag}
                    className="w-full accent-primary"
                  />
                  <span className="text-right tabular-nums">{(pose[index] ?? 0).toFixed(3)}</span>
                </div>
              );
            })}
          </div>
        </Section>

        <Section
          title={t("controller.teach.pointsTitle")}
          hint={t("controller.teach.pointsHint")}
          actions={
            <>
              <Button size="sm" variant="outline" onClick={addPoint} data-testid="add-point">
                {t("controller.teach.addPoint")}
              </Button>
              <Button size="sm" variant="outline" disabled={!armStatus || !status.arms.find((a) => a.arm === `${arm.split("_")[0]}_leader`)?.bound} onClick={() => quietly(addFromLeader())}>
                {t("controller.teach.addFromLeader")}
              </Button>
            </>
          }
        >
          <div className="space-y-1">
            {points.map((point, index) => (
              <div key={index} className="flex items-center gap-1 text-xs">
                <Input className="h-7 w-16" value={point.name} onChange={(e) => editPoint(index, { name: e.target.value })} aria-label={t("controller.teach.pointName")} />
                <Field label="">
                  <Input className="h-7 w-16" type="number" step="0.1" min="0.1" value={point.move_time_s} onChange={(e) => editPoint(index, { move_time_s: Number(e.target.value) })} aria-label={t("controller.teach.moveTime")} />
                </Field>
                <span className="text-muted-foreground">s</span>
                <Button size="icon" variant="ghost" className="h-7 w-7" disabled={index === 0} onClick={() => movePoint(index, -1)} aria-label={t("controller.teach.up")}>
                  <ArrowUp className="h-3.5 w-3.5" />
                </Button>
                <Button size="icon" variant="ghost" className="h-7 w-7" disabled={index === points.length - 1} onClick={() => movePoint(index, 1)} aria-label={t("controller.teach.down")}>
                  <ArrowDown className="h-3.5 w-3.5" />
                </Button>
                <Button size="sm" variant="ghost" className="h-7 px-2" onClick={() => setTarget(point.q)}>
                  {t("controller.teach.show")}
                </Button>
                <Button size="icon" variant="ghost" className="h-7 w-7" onClick={() => setPoints((l) => l.filter((_, i) => i !== index))} aria-label={t("controller.teach.remove")}>
                  <Trash2 className="h-3.5 w-3.5" />
                </Button>
              </div>
            ))}
            {!points.length && <p className="text-xs text-muted-foreground">{t("controller.teach.noPoints")}</p>}
          </div>
          <div className="mt-3 flex flex-wrap gap-2">
            <Button size="sm" variant="outline" disabled={!points.length} onClick={() => (previewing ? stopPreview() : quietly(preview()))}>
              {previewing ? t("controller.teach.stopPreview") : t("controller.teach.preview")}
            </Button>
            <Button size="sm" disabled={!points.length || !canExecute || !!running} onClick={execute} data-testid="execute-program">
              {t("controller.teach.execute")}
            </Button>
          </div>
          <div className="mt-3 flex flex-wrap items-end gap-2">
            <Field label={t("controller.teach.programName")}>
              <Input className="h-8 w-36" value={programName} onChange={(e) => setProgramName(e.target.value)} />
            </Field>
            <Button size="sm" variant="outline" disabled={!points.length} onClick={save}>
              {t("controller.teach.save")}
            </Button>
            <Select onValueChange={load} disabled={!programs.length}>
              <SelectTrigger className="h-8 w-32">
                <SelectValue placeholder={t("controller.teach.load")} />
              </SelectTrigger>
              <SelectContent>
                {programs.map((p) => (
                  <SelectItem key={p.name} value={p.name}>
                    {p.name}
                  </SelectItem>
                ))}
              </SelectContent>
            </Select>
            <Button size="sm" variant="ghost" disabled={!programs.some((p) => p.name === programName)} onClick={remove}>
              {t("controller.teach.delete")}
            </Button>
          </div>
        </Section>
      </div>
    </div>
  );
};

export default TeachPanel;
