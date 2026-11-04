/** 1.7 Replay on the real arm. Offline browsing is a separate page and never moves anything. */
import React, { useEffect, useState } from "react";
import { useTranslation } from "react-i18next";
import { Button } from "@/components/ui/button";
import { Input } from "@/components/ui/input";
import { Select, SelectContent, SelectItem, SelectTrigger, SelectValue } from "@/components/ui/select";
import { quietly, useController } from "@/contexts/ControllerContext";
import { EmptyState, Field, Section, Stat } from "./common";

interface DatasetInfo {
  path: string;
  name: string;
  episodes: number;
}

interface EpisodeInfo {
  episode_index: number;
  length: number;
  tasks: string[];
  result: string;
}

const ReplayPanel: React.FC = () => {
  const { t } = useTranslation();
  const { status, api } = useController();
  const [datasets, setDatasets] = useState<DatasetInfo[]>([]);
  const [dataset, setDataset] = useState("");
  const [episodes, setEpisodes] = useState<EpisodeInfo[]>([]);
  const [episode, setEpisode] = useState("");
  const [start, setStart] = useState("");
  const [end, setEnd] = useState("");
  const [speed, setSpeed] = useState("1.0");
  const [blend, setBlend] = useState("2.0");
  const [fps, setFps] = useState(30);
  const [column, setColumn] = useState("action");
  const [programs, setPrograms] = useState<{ name: string; points: number }[]>([]);
  const [program, setProgram] = useState("");

  useEffect(() => {
    api<DatasetInfo[]>("/data/datasets", undefined, { quiet: true }).then(setDatasets).catch(() => undefined);
    api<{ name: string; points: number }[]>("/programs", undefined, { quiet: true }).then(setPrograms).catch(() => undefined);
  }, [api]);

  useEffect(() => {
    if (!dataset) return;
    setEpisode("");
    api<EpisodeInfo[]>(`/data/episodes?root=${encodeURIComponent(dataset)}`, undefined, { quiet: true }).then(setEpisodes).catch(() => setEpisodes([]));
    const info = datasets.find((d) => d.path === dataset) as (DatasetInfo & { fps?: number }) | undefined;
    if (info?.fps) setFps(info.fps);
  }, [api, dataset, datasets]);

  if (!status?.connected) return <EmptyState>{t("controller.common.connectFirst")}</EmptyState>;

  const chosen = episodes.find((e) => String(e.episode_index) === episode);
  const duration = chosen ? chosen.length / fps : 0;
  const idle = status.source === "none" && status.estop === "CLEAR";
  const running = status.source === "replay";
  const progress = status.execution?.arms[0];

  const replay = () =>
    quietly(
      api("/replay/episode", {
        root: dataset,
        episode: Number(episode),
        start_s: start === "" ? null : Number(start),
        end_s: end === "" ? null : Number(end),
        speed: Number(speed),
        column,
        blend_s: Number(blend),
      }),
    );

  const replayProgram = async () => {
    const data = await api<{ name: string; arm: string; model: string; points: unknown[] }>(`/programs/${encodeURIComponent(program)}`);
    await api("/programs/execute", { program: data, dt_s: 0.02 });
  };

  return (
    <div className="grid gap-4 xl:grid-cols-2">
      <Section title={t("controller.replay.episodeTitle")} hint={t("controller.replay.episodeHint")}>
        <div className="grid grid-cols-2 gap-3 sm:grid-cols-3">
          <Field label={t("controller.replay.dataset")} className="col-span-2">
            <Select value={dataset} onValueChange={setDataset}>
              <SelectTrigger className="h-8">
                <SelectValue placeholder={t("controller.replay.choose")} />
              </SelectTrigger>
              <SelectContent>
                {datasets.map((d) => (
                  <SelectItem key={d.path} value={d.path}>
                    {d.name} ({d.episodes})
                  </SelectItem>
                ))}
              </SelectContent>
            </Select>
          </Field>
          <Field label={t("controller.replay.episode")}>
            <Select value={episode} onValueChange={setEpisode} disabled={!episodes.length}>
              <SelectTrigger className="h-8">
                <SelectValue placeholder="-" />
              </SelectTrigger>
              <SelectContent>
                {episodes.map((e) => (
                  <SelectItem key={e.episode_index} value={String(e.episode_index)}>
                    #{e.episode_index} {t(`controller.result.${e.result}` as never)}
                  </SelectItem>
                ))}
              </SelectContent>
            </Select>
          </Field>
          <Field label={t("controller.replay.start")}>
            <Input className="h-8" value={start} onChange={(e) => setStart(e.target.value)} placeholder="0" inputMode="decimal" />
          </Field>
          <Field label={t("controller.replay.end")}>
            <Input className="h-8" value={end} onChange={(e) => setEnd(e.target.value)} placeholder={duration ? duration.toFixed(2) : ""} inputMode="decimal" />
          </Field>
          <Field label={t("controller.replay.speed")}>
            <Input className="h-8" value={speed} onChange={(e) => setSpeed(e.target.value)} inputMode="decimal" />
          </Field>
          <Field label={t("controller.replay.approach")}>
            <Input className="h-8" value={blend} onChange={(e) => setBlend(e.target.value)} inputMode="decimal" />
          </Field>
          <Field label={t("controller.replay.column")}>
            <Select value={column} onValueChange={setColumn}>
              <SelectTrigger className="h-8">
                <SelectValue />
              </SelectTrigger>
              <SelectContent>
                <SelectItem value="action">{t("controller.replay.columnAction")}</SelectItem>
                <SelectItem value="state">{t("controller.replay.columnState")}</SelectItem>
              </SelectContent>
            </Select>
          </Field>
        </div>
        <div className="mt-4 flex flex-wrap gap-2">
          <Button size="sm" disabled={!idle || !dataset || episode === ""} onClick={replay} data-testid="replay-start">
            {t("controller.replay.play")}
          </Button>
          <Button size="sm" variant="outline" disabled={!running || status.paused} onClick={() => quietly(api("/run/pause", {}))}>
            {t("controller.run.pause")}
          </Button>
          <Button size="sm" variant="outline" disabled={!status.paused} onClick={() => quietly(api("/run/resume", {}))}>
            {t("controller.run.resume")}
          </Button>
        </div>
      </Section>
      <div className="space-y-4">
        <Section title={t("controller.replay.programTitle")}>
          <div className="flex flex-wrap items-end gap-2">
            <Field label={t("controller.replay.program")}>
              <Select value={program} onValueChange={setProgram} disabled={!programs.length}>
                <SelectTrigger className="h-8 w-48">
                  <SelectValue placeholder={t("controller.replay.choose")} />
                </SelectTrigger>
                <SelectContent>
                  {programs.map((p) => (
                    <SelectItem key={p.name} value={p.name}>
                      {p.name} ({p.points})
                    </SelectItem>
                  ))}
                </SelectContent>
              </Select>
            </Field>
            <Button size="sm" disabled={!idle || !program} onClick={() => quietly(replayProgram())}>
              {t("controller.replay.play")}
            </Button>
          </div>
        </Section>
        <Section title={t("controller.replay.progressTitle")} hint={t("controller.replay.progressHint")}>
          <div className="grid grid-cols-2 gap-3">
            <Stat label={t("controller.replay.executed")} value={progress ? `${progress.executed} / ${progress.total}` : "-"} />
            <Stat label={t("controller.replay.remaining")} value={progress && status.execution ? `${(((progress.total - progress.executed) * status.execution.dt_s) || 0).toFixed(1)} s` : "-"} />
          </div>
          {progress && (
            <div className="mt-3 h-2 overflow-hidden rounded bg-muted">
              <div className="h-full bg-primary" style={{ width: `${(100 * progress.executed) / Math.max(1, progress.total)}%` }} />
            </div>
          )}
        </Section>
      </div>
    </div>
  );
};

export default ReplayPanel;
