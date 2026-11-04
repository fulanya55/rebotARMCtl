/** 2.1 and 2.3 Recording: independent of who controls the arms, written in the background. */
import React, { useEffect, useState } from "react";
import { useTranslation } from "react-i18next";
import { Button } from "@/components/ui/button";
import { Input } from "@/components/ui/input";
import { Badge } from "@/components/ui/badge";
import { Switch } from "@/components/ui/switch";
import { Select, SelectContent, SelectItem, SelectTrigger, SelectValue } from "@/components/ui/select";
import { quietly, useController } from "@/contexts/ControllerContext";
import { EmptyState, Field, Section, Stat } from "./common";

interface LiveStats {
  elapsed_s: number;
  arms: Record<string, { samples: number; rate_hz: number; missing: number }>;
  cameras: Record<string, { frames: number; rate_hz: number; dropped: number; error: string | null }>;
}

interface RecordStatus {
  recording: boolean;
  pending_save: boolean;
  live: LiveStats | null;
  finished: LiveStats | null;
  writer: {
    pending: number;
    jobs: { job: string; state: string; error: string | null; episode_index: number | null; progress: number }[];
  };
}

const RecordPanel: React.FC = () => {
  const { t } = useTranslation();
  const { status, api } = useController();
  const [root, setRoot] = useState("");
  const [name, setName] = useState("rac_dataset");
  const [task, setTask] = useState("");
  const [fps, setFps] = useState("30");
  const [cameras, setCameras] = useState<string[]>([]);
  const [useLeft, setUseLeft] = useState(true);
  const [useRight, setUseRight] = useState(false);
  const [state, setState] = useState<RecordStatus | null>(null);
  const [result, setResult] = useState("unlabeled");
  const [note, setNote] = useState("");

  useEffect(() => {
    api<{ root: string }>("/data/root", undefined, { quiet: true }).then((r) => setRoot(r.root)).catch(() => undefined);
    api<{ running: boolean; config: Record<string, unknown> }>("/cameras", undefined, { quiet: true })
      .then((c) => {
        if (c.running) setCameras(Object.keys(c.config));
      })
      .catch(() => undefined);
  }, [api]);

  useEffect(() => {
    const poll = () => api<RecordStatus>("/record/status", undefined, { quiet: true }).then(setState).catch(() => undefined);
    void poll();
    const timer = setInterval(() => void poll(), 500);
    return () => clearInterval(timer);
  }, [api]);

  if (!status?.connected) return <EmptyState>{t("controller.common.connectFirst")}</EmptyState>;

  const recording = !!state?.recording;
  const pending = !!state?.pending_save;
  const stats = state?.live ?? state?.finished ?? null;

  const body = () => ({
    root: `${root.replace(/[\\/]+$/, "")}/${name}`,
    repo_id: `local/${name}`,
    task,
    fps: Number(fps),
    cameras,
    follower_arms: [useLeft && "left_follower", useRight && "right_follower"].filter(Boolean),
    leader_arms: [useLeft && "left_leader", useRight && "right_leader"].filter(Boolean),
  });

  const start = () => quietly(api("/record/start", body()));
  const save = () => quietly(api("/record/save", { result, note }).then(() => setNote("")));

  return (
    <div className="grid gap-4 xl:grid-cols-2">
      <Section title={t("controller.record.setupTitle")} hint={t("controller.record.setupHint")}>
        <fieldset disabled={recording} className="grid grid-cols-2 gap-3 sm:grid-cols-3">
          <Field label={t("controller.record.root")} className="col-span-2">
            <Input className="h-8" value={root} onChange={(e) => setRoot(e.target.value)} />
          </Field>
          <Field label={t("controller.record.dataset")}>
            <Input className="h-8" value={name} onChange={(e) => setName(e.target.value)} />
          </Field>
          <Field label={t("controller.record.task")} className="col-span-2">
            <Input className="h-8" value={task} onChange={(e) => setTask(e.target.value)} placeholder={t("controller.policy.taskHint")} data-testid="record-task" />
          </Field>
          <Field label={t("controller.record.fps")}>
            <Select value={fps} onValueChange={setFps} disabled={recording}>
              <SelectTrigger className="h-8">
                <SelectValue />
              </SelectTrigger>
              <SelectContent>
                {["15", "30", "60"].map((value) => (
                  <SelectItem key={value} value={value}>
                    {value}
                  </SelectItem>
                ))}
              </SelectContent>
            </Select>
          </Field>
          <Field label={t("controller.record.cameras")} className="col-span-2">
            <Input className="h-8" value={cameras.join(",")} onChange={(e) => setCameras(e.target.value.split(",").map((s) => s.trim()).filter(Boolean))} />
          </Field>
          <div className="col-span-2 flex items-center gap-4 sm:col-span-3">
            <label className="flex items-center gap-1.5 text-xs">
              <Switch checked={useLeft} onCheckedChange={setUseLeft} disabled={recording} />
              {t("controller.teleop.side_left")}
            </label>
            <label className="flex items-center gap-1.5 text-xs">
              <Switch checked={useRight} onCheckedChange={setUseRight} disabled={recording} />
              {t("controller.teleop.side_right")}
            </label>
          </div>
        </fieldset>
        <div className="mt-4 flex flex-wrap gap-2">
          <Button size="sm" disabled={recording || pending || !task || !cameras.length} onClick={start} data-testid="record-start">
            {t("controller.record.start")}
          </Button>
          <Button size="sm" variant="outline" disabled={!recording} onClick={() => quietly(api("/record/stop", {}))} data-testid="record-stop">
            {t("controller.record.stop")}
          </Button>
          <Button size="sm" variant="outline" disabled={!recording} onClick={() => quietly(api("/record/next", { result, note }))}>
            {t("controller.record.next")}
          </Button>
        </div>
        <p className="mt-2 text-xs text-muted-foreground">{t("controller.record.appendNote")}</p>
      </Section>

      <div className="space-y-4">
        <Section title={t("controller.record.statusTitle")} actions={<Badge variant={recording ? "destructive" : "secondary"}>{recording ? t("controller.record.recording") : pending ? t("controller.record.pending") : t("controller.record.idle")}</Badge>}>
          {stats ? (
            <>
              <Stat label={t("controller.record.elapsed")} value={`${stats.elapsed_s.toFixed(1)} s`} />
              <table className="mt-3 w-full text-xs tabular-nums">
                <thead className="text-muted-foreground">
                  <tr>
                    <th className="py-1 text-left font-normal">{t("controller.record.source")}</th>
                    <th className="py-1 text-right font-normal">{t("controller.record.count")}</th>
                    <th className="py-1 text-right font-normal">{t("controller.record.rate")}</th>
                    <th className="py-1 text-right font-normal">{t("controller.record.missing")}</th>
                  </tr>
                </thead>
                <tbody>
                  {Object.entries(stats.cameras).map(([camera, c]) => (
                    <tr key={camera} className={c.error ? "text-destructive" : ""}>
                      <td className="py-0.5">{camera}</td>
                      <td className="py-0.5 text-right">{c.frames}</td>
                      <td className="py-0.5 text-right">{c.rate_hz}</td>
                      <td className="py-0.5 text-right">{c.dropped}</td>
                    </tr>
                  ))}
                  {Object.entries(stats.arms).map(([arm, a]) => (
                    <tr key={arm}>
                      <td className="py-0.5">{t(`controller.arm.${arm}` as never)}</td>
                      <td className="py-0.5 text-right">{a.samples}</td>
                      <td className="py-0.5 text-right">{a.rate_hz}</td>
                      <td className="py-0.5 text-right">{a.missing}</td>
                    </tr>
                  ))}
                </tbody>
              </table>
            </>
          ) : (
            <p className="text-xs text-muted-foreground">{t("controller.record.notStarted")}</p>
          )}
        </Section>

        {pending && (
          <Section title={t("controller.record.saveTitle")} hint={t("controller.record.saveHint")}>
            <div className="flex flex-wrap items-end gap-2">
              <Field label={t("controller.data.result")}>
                <Select value={result} onValueChange={setResult}>
                  <SelectTrigger className="h-8 w-28">
                    <SelectValue />
                  </SelectTrigger>
                  <SelectContent>
                    {["success", "failure", "incomplete", "unlabeled"].map((value) => (
                      <SelectItem key={value} value={value}>
                        {t(`controller.result.${value}` as never)}
                      </SelectItem>
                    ))}
                  </SelectContent>
                </Select>
              </Field>
              <Field label={t("controller.data.note")} className="min-w-40 flex-1">
                <Input className="h-8" value={note} onChange={(e) => setNote(e.target.value)} />
              </Field>
              <Button size="sm" onClick={save} data-testid="record-save">
                {t("controller.record.save")}
              </Button>
              <Button size="sm" variant="outline" onClick={() => quietly(api("/record/discard", {}))} data-testid="record-discard">
                {t("controller.record.discard")}
              </Button>
            </div>
          </Section>
        )}

        <Section title={t("controller.record.writerTitle")} hint={t("controller.record.writerHint")}>
          {state?.writer.jobs.length ? (
            <ul className="space-y-1 text-xs">
              {state.writer.jobs
                .slice()
                .reverse()
                .map((job) => (
                  <li key={job.job} className="flex flex-wrap items-center gap-2" data-testid="writer-job" data-state={job.state}>
                    <Badge variant={job.state === "failed" ? "destructive" : job.state === "done" ? "default" : "secondary"}>{t(`controller.record.job_${job.state}` as never)}</Badge>
                    {job.episode_index !== null && <span>#{job.episode_index}</span>}
                    {job.state === "writing" && <span className="tabular-nums">{Math.round(job.progress * 100)}%</span>}
                    {job.error && <span className="text-destructive">{job.error}</span>}
                  </li>
                ))}
            </ul>
          ) : (
            <p className="text-xs text-muted-foreground">{t("controller.record.noJobs")}</p>
          )}
        </Section>
      </div>
    </div>
  );
};

export default RecordPanel;
