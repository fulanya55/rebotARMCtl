/** 1.5 Deployment and 1.6 intervention: a model in its own process, a person one click away. */
import React, { useState } from "react";
import { useTranslation } from "react-i18next";
import { Button } from "@/components/ui/button";
import { Input } from "@/components/ui/input";
import { Badge } from "@/components/ui/badge";
import { Switch } from "@/components/ui/switch";
import { Select, SelectContent, SelectItem, SelectTrigger, SelectValue } from "@/components/ui/select";
import { quietly, useController } from "@/contexts/ControllerContext";
import { EmptyState, Field, Section, Stat } from "./common";

const PolicyPanel: React.FC = () => {
  const { t } = useTranslation();
  const { status, api } = useController();
  const [backend, setBackend] = useState("mock");
  const [checkpoint, setCheckpoint] = useState("");
  const [host, setHost] = useState("127.0.0.1");
  const [port, setPort] = useState("10093");
  const [unnormKey, setUnnormKey] = useState("");
  const [task, setTask] = useState("");
  const [inferenceHz, setInferenceHz] = useState("5");
  const [actionHz, setActionHz] = useState("30");
  const [strategy, setStrategy] = useState("stream");
  const [units, setUnits] = useState("none");
  const [useLeft, setUseLeft] = useState(true);
  const [useRight, setUseRight] = useState(false);
  const [cameras, setCameras] = useState("env=env,wrist=wrist");
  const [follow, setFollow] = useState("relative");
  const [markNote, setMarkNote] = useState("");

  if (!status?.connected) return <EmptyState>{t("controller.common.connectFirst")}</EmptyState>;

  const policy = status.policy;
  const running = policy.state === "running" || policy.state === "paused" || policy.state === "suspended";
  const intervention = status.intervention.active;
  const aligned = status.pairs.some((p) => p.phase === "ALIGNED");

  const start = () => {
    const arms = [useLeft && "left_follower", useRight && "right_follower"].filter(Boolean);
    const cameraMap = Object.fromEntries(
      cameras
        .split(",")
        .map((pair) => pair.trim().split("="))
        .filter((pair) => pair.length === 2 && pair[0] && pair[1]),
    );
    quietly(
      api("/policy/start", {
        config: {
          backend,
          arms,
          cameras: cameraMap,
          task,
          inference_hz: Number(inferenceHz),
          action_hz: Number(actionHz),
          strategy,
          state_norm: { mode: units },
          action_norm: { mode: units },
          checkpoint,
          host,
          port: Number(port),
          unnorm_key: unnormKey || undefined,
        },
      }),
    );
  };

  return (
    <div className="grid gap-4 xl:grid-cols-2">
      <Section title={t("controller.policy.setupTitle")} hint={t("controller.policy.setupHint")}>
        <fieldset disabled={running} className="grid grid-cols-2 gap-3 sm:grid-cols-3">
          <Field label={t("controller.policy.backend")}>
            <Select value={backend} onValueChange={setBackend} disabled={running}>
              <SelectTrigger className="h-8">
                <SelectValue />
              </SelectTrigger>
              <SelectContent>
                <SelectItem value="lerobot">LeRobot</SelectItem>
                <SelectItem value="starvla">StarVLA</SelectItem>
                <SelectItem value="mock">{t("controller.policy.mock")}</SelectItem>
              </SelectContent>
            </Select>
          </Field>
          {backend === "lerobot" && (
            <Field label={t("controller.policy.checkpoint")} className="col-span-2">
              <Input className="h-8" value={checkpoint} onChange={(e) => setCheckpoint(e.target.value)} placeholder="outputs/train/.../pretrained_model" />
            </Field>
          )}
          {backend === "starvla" && (
            <>
              <Field label={t("controller.policy.host")}>
                <Input className="h-8" value={host} onChange={(e) => setHost(e.target.value)} />
              </Field>
              <Field label={t("controller.policy.port")}>
                <Input className="h-8" value={port} onChange={(e) => setPort(e.target.value)} />
              </Field>
              <Field label={t("controller.policy.unnormKey")}>
                <Input className="h-8" value={unnormKey} onChange={(e) => setUnnormKey(e.target.value)} />
              </Field>
            </>
          )}
          <Field label={t("controller.policy.task")} className="col-span-2 sm:col-span-3">
            <Input className="h-8" value={task} onChange={(e) => setTask(e.target.value)} placeholder={t("controller.policy.taskHint")} />
          </Field>
          <Field label={t("controller.policy.inferenceHz")}>
            <Input className="h-8" value={inferenceHz} onChange={(e) => setInferenceHz(e.target.value)} inputMode="decimal" />
          </Field>
          <Field label={t("controller.policy.actionHz")}>
            <Input className="h-8" value={actionHz} onChange={(e) => setActionHz(e.target.value)} inputMode="decimal" />
          </Field>
          <Field label={t("controller.policy.strategy")}>
            <Select value={strategy} onValueChange={setStrategy} disabled={running}>
              <SelectTrigger className="h-8">
                <SelectValue />
              </SelectTrigger>
              <SelectContent>
                <SelectItem value="stream">{t("controller.policy.strategyStream")}</SelectItem>
                <SelectItem value="naive">{t("controller.policy.strategyNaive")}</SelectItem>
                <SelectItem value="ensemble">{t("controller.policy.strategyEnsemble")}</SelectItem>
              </SelectContent>
            </Select>
          </Field>
          <Field label={t("controller.policy.units")}>
            <Select value={units} onValueChange={setUnits} disabled={running}>
              <SelectTrigger className="h-8">
                <SelectValue />
              </SelectTrigger>
              <SelectContent>
                <SelectItem value="none">{t("controller.policy.unitsRadians")}</SelectItem>
                <SelectItem value="degrees">{t("controller.policy.unitsDegrees")}</SelectItem>
              </SelectContent>
            </Select>
          </Field>
          <Field label={t("controller.policy.cameras")} className="col-span-2">
            <Input className="h-8" value={cameras} onChange={(e) => setCameras(e.target.value)} />
          </Field>
          <div className="col-span-2 flex items-center gap-4 sm:col-span-3">
            <label className="flex items-center gap-1.5 text-xs">
              <Switch checked={useLeft} onCheckedChange={setUseLeft} disabled={running} />
              {t("controller.arm.left_follower")}
            </label>
            <label className="flex items-center gap-1.5 text-xs">
              <Switch checked={useRight} onCheckedChange={setUseRight} disabled={running} />
              {t("controller.arm.right_follower")}
            </label>
          </div>
        </fieldset>
        <div className="mt-4 flex flex-wrap gap-2">
          <Button size="sm" disabled={running || status.source !== "none"} onClick={start} data-testid="policy-start">
            {t("controller.policy.start")}
          </Button>
          <Button size="sm" variant="outline" disabled={policy.state !== "running"} onClick={() => quietly(api("/policy/pause", {}))}>
            {t("controller.policy.pause")}
          </Button>
          <Button size="sm" variant="outline" disabled={policy.state !== "paused"} onClick={() => quietly(api("/policy/resume", {}))}>
            {t("controller.policy.resume")}
          </Button>
          <Button size="sm" variant="outline" disabled={!running} onClick={() => quietly(api("/policy/stop", {}))} data-testid="policy-stop">
            {t("controller.policy.stop")}
          </Button>
        </div>
      </Section>

      <div className="space-y-4">
        <Section title={t("controller.policy.statusTitle")} actions={<Badge data-testid="policy-state">{t(`controller.policyState.${policy.state}` as never)}</Badge>}>
          <div className="grid grid-cols-2 gap-3 sm:grid-cols-3">
            <Stat label={t("controller.policy.inferences")} value={policy.stats.inferences ?? 0} />
            <Stat label={t("controller.policy.latency")} value={policy.stats.last_inference_ms != null ? `${policy.stats.last_inference_ms.toFixed(1)} ms` : "-"} />
            <Stat label={t("controller.policy.actionsSent")} value={policy.stats.actions_sent ?? 0} />
            <Stat label={t("controller.policy.buffered")} value={policy.buffered} />
            <Stat label={t("controller.policy.rejected")} value={policy.stats.rejected ?? 0} />
            <Stat label={t("controller.policy.process")} value={policy.process_alive ? t("controller.policy.alive") : "-"} />
          </div>
        </Section>

        <Section title={t("controller.policy.interventionTitle")} hint={t("controller.policy.interventionHint")}>
          <div className="flex flex-wrap items-end gap-2">
            <Field label={t("controller.policy.takeoverMode")}>
              <Select value={follow} onValueChange={setFollow}>
                <SelectTrigger className="h-8 w-40">
                  <SelectValue />
                </SelectTrigger>
                <SelectContent>
                  <SelectItem value="relative">{t("controller.teleop.relative")}</SelectItem>
                  <SelectItem value="align">{t("controller.policy.alignFirst")}</SelectItem>
                </SelectContent>
              </Select>
            </Field>
            <Button size="sm" variant="destructive" disabled={!(policy.state === "running" || policy.state === "paused")} onClick={() => quietly(api("/intervention/take-over", { follow }))} data-testid="take-over">
              {t("controller.policy.takeOver")}
            </Button>
            {intervention && follow === "align" && (
              <Button size="sm" variant="outline" disabled={!aligned} onClick={() => quietly(api("/intervention/confirm-aligned", {}))}>
                {t("controller.policy.confirmAligned")}
              </Button>
            )}
            <Button size="sm" disabled={!intervention} onClick={() => quietly(api("/intervention/hand-back", {}))} data-testid="hand-back">
              {t("controller.policy.handBack")}
            </Button>
          </div>
          <div className="mt-3 flex flex-wrap items-end gap-2">
            <Field label={t("controller.policy.correctionNote")} className="min-w-48 flex-1">
              <Input className="h-8" value={markNote} onChange={(e) => setMarkNote(e.target.value)} />
            </Field>
            <Button size="sm" variant="outline" disabled={!intervention || !markNote} onClick={() => quietly(api("/intervention/mark", { label: "correction", note: markNote }).then(() => setMarkNote("")))}>
              {t("controller.policy.mark")}
            </Button>
          </div>
          {status.intervention.segments.length > 0 && (
            <p className="mt-3 text-xs text-muted-foreground" data-testid="intervention-count">
              {t("controller.policy.segments", { count: status.intervention.segments.length })}
            </p>
          )}
        </Section>
      </div>
    </div>
  );
};

export default PolicyPanel;
