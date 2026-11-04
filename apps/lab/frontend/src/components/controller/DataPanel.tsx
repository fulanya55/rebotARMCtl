/**
 * 2.3 to 2.5 Data browser and annotation. Offline: it updates this page and nothing
 * else, and never moves an arm. Physical replay is started from the replay page.
 */
import React, { useCallback, useEffect, useMemo, useRef, useState } from "react";
import { useTranslation } from "react-i18next";
import { CartesianGrid, Line, LineChart, ReferenceArea, ReferenceLine, ResponsiveContainer, Tooltip, XAxis, YAxis } from "recharts";
import { Button } from "@/components/ui/button";
import { Input } from "@/components/ui/input";
import { Badge } from "@/components/ui/badge";
import { toast } from "@/components/ui/sonner";
import { Select, SelectContent, SelectItem, SelectTrigger, SelectValue } from "@/components/ui/select";
import { ChevronLeft, ChevronRight, Pause, Play } from "lucide-react";
import { quietly, useController } from "@/contexts/ControllerContext";
import { useApi } from "@/contexts/ApiContext";
import { EmptyState, Field, Section } from "./common";

interface DatasetInfo {
  path: string;
  name: string;
  episodes: number;
  frames: number;
  fps: number;
  codebase_version: string;
}

interface Interval {
  start_s: number;
  end_s: number;
  source: number;
  intervention: boolean;
}

interface SegmentInfo {
  id: number;
  start_s: number;
  end_s: number;
  label: string;
  note: string;
  kind: string;
}

interface EpisodeRow {
  episode_index: number;
  length: number;
  tasks: string[];
  result: string;
  note: string;
  sources: string[];
  control_intervals: Interval[];
  segments: SegmentInfo[];
  excluded: boolean;
  quality?: {
    frames: number;
    invalid_frames: number;
    followers: { rate_hz: number; missing: number }[];
    cameras: Record<string, { dropped_by_camera: number; late_frames: number }>;
  };
}

interface EpisodeData {
  fps: number;
  length: number;
  joints: string[];
  timestamp: number[];
  state: number[][];
  action: number[][];
  leader: number[][];
  control_source: number[];
  intervention: number[];
  videos: Record<string, { path: string; from_s: number; to_s: number }>;
}

const SOURCE_BY_ID = ["none", "teleop", "teach", "policy", "intervention", "replay", "home", "guide"];

const SERIES = "#4aa8ff";
const ACTION = "#ff9d4a";

const DataPanel: React.FC = () => {
  const { t } = useTranslation();
  const { api } = useController();
  const { baseUrl } = useApi();
  const [base, setBase] = useState("");
  const [datasets, setDatasets] = useState<DatasetInfo[]>([]);
  const [dataset, setDataset] = useState("");
  const [episodes, setEpisodes] = useState<EpisodeRow[]>([]);
  const [filterResult, setFilterResult] = useState("all");
  const [filterSource, setFilterSource] = useState("all");
  const [filterTask, setFilterTask] = useState("");
  const [selected, setSelected] = useState<number | null>(null);
  const [data, setData] = useState<EpisodeData | null>(null);
  const [time, setTime] = useState(0);
  const [playing, setPlaying] = useState(false);
  const [rate, setRate] = useState(1);
  const [joint, setJoint] = useState(0);
  const videos = useRef<Record<string, HTMLVideoElement | null>>({});
  const raf = useRef(0);
  const lastTick = useRef(0);

  const [segStart, setSegStart] = useState("");
  const [segEnd, setSegEnd] = useState("");
  const [segLabel, setSegLabel] = useState("");
  const [segKind, setSegKind] = useState("phase");
  const [segNote, setSegNote] = useState("");
  const [mergeOutput, setMergeOutput] = useState("");
  const [mergeWith, setMergeWith] = useState<string[]>([]);
  const [exportPath, setExportPath] = useState("");
  const [importPath, setImportPath] = useState("");

  const loadDatasets = useCallback(
    (root?: string) => {
      const query = root ? `?base=${encodeURIComponent(root)}` : "";
      api<DatasetInfo[]>(`/data/datasets${query}`, undefined, { quiet: true }).then(setDatasets).catch(() => setDatasets([]));
    },
    [api],
  );

  useEffect(() => {
    api<{ root: string }>("/data/root", undefined, { quiet: true })
      .then((r) => {
        setBase(r.root);
        loadDatasets(r.root);
      })
      .catch(() => undefined);
  }, [api, loadDatasets]);

  const loadEpisodes = useCallback(() => {
    if (!dataset) return;
    const params = new URLSearchParams({ root: dataset });
    if (filterResult !== "all") params.set("result", filterResult);
    if (filterSource !== "all") params.set("source", filterSource);
    if (filterTask) params.set("task", filterTask);
    api<EpisodeRow[]>(`/data/episodes?${params}`, undefined, { quiet: true }).then(setEpisodes).catch(() => setEpisodes([]));
  }, [api, dataset, filterResult, filterSource, filterTask]);
  useEffect(loadEpisodes, [loadEpisodes]);

  useEffect(() => {
    if (selected === null || !dataset) {
      setData(null);
      return;
    }
    setPlaying(false);
    setTime(0);
    api<EpisodeData>(`/data/episode?root=${encodeURIComponent(dataset)}&episode=${selected}`, undefined, { quiet: true })
      .then(setData)
      .catch(() => setData(null));
  }, [api, dataset, selected]);

  const duration = data ? data.length / data.fps : 0;
  const row = episodes.find((e) => e.episode_index === selected);

  // Videos of one dataset file hold many episodes: each is positioned by its offset.
  const seek = useCallback(
    (next: number) => {
      if (!data) return;
      const clamped = Math.max(0, Math.min(next, duration));
      setTime(clamped);
      for (const [name, info] of Object.entries(data.videos)) {
        const video = videos.current[name];
        if (video) video.currentTime = info.from_s + clamped;
      }
    },
    [data, duration],
  );

  useEffect(() => {
    if (!playing || !data) return;
    lastTick.current = performance.now();
    for (const video of Object.values(videos.current)) {
      if (video) {
        video.playbackRate = rate;
        void video.play().catch(() => undefined);
      }
    }
    const tick = (now: number) => {
      const delta = ((now - lastTick.current) / 1000) * rate;
      lastTick.current = now;
      setTime((current) => {
        const next = current + delta;
        if (next >= duration) {
          setPlaying(false);
          return duration;
        }
        return next;
      });
      raf.current = requestAnimationFrame(tick);
    };
    raf.current = requestAnimationFrame(tick);
    return () => {
      cancelAnimationFrame(raf.current);
      for (const video of Object.values(videos.current)) video?.pause();
    };
  }, [playing, rate, data, duration]);

  const frame = data ? Math.min(data.length - 1, Math.round(time * data.fps)) : 0;
  const stepFrame = (delta: number) => {
    if (!data) return;
    setPlaying(false);
    seek((frame + delta) / data.fps);
  };

  const chart = useMemo(() => {
    if (!data) return [];
    const stride = Math.max(1, Math.floor(data.length / 600));
    const out: { t: number; state: number; action: number; leader: number }[] = [];
    for (let i = 0; i < data.length; i += stride) {
      out.push({ t: data.timestamp[i], state: data.state[i][joint], action: data.action[i][joint], leader: data.leader[i][joint] });
    }
    return out;
  }, [data, joint]);

  const annotate = (fields: Record<string, unknown>) => quietly(api("/data/annotate", { root: dataset, episode: selected, ...fields }).then(loadEpisodes));

  const addSegment = () =>
    quietly(
      api("/data/segments", {
        root: dataset,
        episode: selected,
        start_s: Number(segStart),
        end_s: Number(segEnd),
        label: segLabel,
        note: segNote,
        kind: segKind,
      }).then(() => {
        setSegLabel("");
        setSegNote("");
        loadEpisodes();
      }),
    );

  const removeSegment = (segment: number) => quietly(api("/data/segments/delete", { root: dataset, episode: selected, segment, fields: {} }).then(loadEpisodes));

  const merge = async () => {
    const roots = [dataset, ...mergeWith];
    const check = await api<{ ok: boolean; problems: string[] }>("/data/check-merge", { roots, output: mergeOutput });
    if (!check.ok) {
      toast.error(check.problems.join("; "));
      return;
    }
    await api("/data/merge", { roots, output: mergeOutput });
    toast.success(t("controller.data.merged"));
    loadDatasets(base);
  };

  return (
    <div className="space-y-4">
      <Section title={t("controller.data.datasetsTitle")} hint={t("controller.data.datasetsHint")}>
        <div className="flex flex-wrap items-end gap-2">
          <Field label={t("controller.data.base")} className="min-w-64 flex-1">
            <Input className="h-8" value={base} onChange={(e) => setBase(e.target.value)} />
          </Field>
          <Button size="sm" variant="outline" onClick={() => loadDatasets(base)}>
            {t("controller.data.refresh")}
          </Button>
          <Field label={t("controller.data.dataset")}>
            <Select
              value={dataset}
              onValueChange={(value) => {
                setDataset(value);
                setSelected(null);
              }}
            >
              <SelectTrigger className="h-8 w-56" data-testid="dataset-select">
                <SelectValue placeholder={t("controller.replay.choose")} />
              </SelectTrigger>
              <SelectContent>
                {datasets.map((d) => (
                  <SelectItem key={d.path} value={d.path}>
                    {d.name} · {d.episodes} · {d.fps}Hz
                  </SelectItem>
                ))}
              </SelectContent>
            </Select>
          </Field>
        </div>
      </Section>

      {dataset && (
        <div className="grid gap-4 xl:grid-cols-[300px_minmax(0,1fr)]">
          <Section title={t("controller.data.episodesTitle")}>
            <div className="mb-2 grid grid-cols-2 gap-2">
              <Select value={filterResult} onValueChange={setFilterResult}>
                <SelectTrigger className="h-8">
                  <SelectValue />
                </SelectTrigger>
                <SelectContent>
                  <SelectItem value="all">{t("controller.data.allResults")}</SelectItem>
                  {["success", "failure", "incomplete", "unlabeled"].map((value) => (
                    <SelectItem key={value} value={value}>
                      {t(`controller.result.${value}` as never)}
                    </SelectItem>
                  ))}
                </SelectContent>
              </Select>
              <Select value={filterSource} onValueChange={setFilterSource}>
                <SelectTrigger className="h-8">
                  <SelectValue />
                </SelectTrigger>
                <SelectContent>
                  <SelectItem value="all">{t("controller.data.allSources")}</SelectItem>
                  {["teleop", "policy", "intervention", "teach", "replay", "guide"].map((value) => (
                    <SelectItem key={value} value={value}>
                      {t(`controller.source.${value}` as never)}
                    </SelectItem>
                  ))}
                </SelectContent>
              </Select>
              <Input className="col-span-2 h-8" value={filterTask} onChange={(e) => setFilterTask(e.target.value)} placeholder={t("controller.data.filterTask")} />
            </div>
            <ul className="max-h-[420px] space-y-1 overflow-y-auto" data-testid="episode-list">
              {episodes.map((episode) => (
                <li key={episode.episode_index}>
                  <button
                    className={`w-full rounded border px-2 py-1.5 text-left text-xs ${selected === episode.episode_index ? "border-primary bg-primary/10" : "hover:bg-muted"} ${episode.excluded ? "opacity-50" : ""}`}
                    onClick={() => setSelected(episode.episode_index)}
                  >
                    <div className="flex items-center justify-between gap-2">
                      <span className="font-medium">#{episode.episode_index}</span>
                      <Badge variant={episode.result === "success" ? "default" : episode.result === "failure" ? "destructive" : "secondary"}>{t(`controller.result.${episode.result}` as never)}</Badge>
                    </div>
                    <div className="truncate text-muted-foreground">{episode.tasks[0]}</div>
                    <div className="text-muted-foreground">{episode.sources.map((s) => t(`controller.source.${s}` as never)).join(" · ")}</div>
                  </button>
                </li>
              ))}
              {!episodes.length && <li className="py-4 text-center text-xs text-muted-foreground">{t("controller.data.noEpisodes")}</li>}
            </ul>
          </Section>

          {data && row ? (
            <div className="space-y-4">
              <Section title={t("controller.data.viewerTitle", { index: selected ?? 0 })}>
                <div className="grid gap-2 sm:grid-cols-2">
                  {Object.entries(data.videos).map(([name, info]) => (
                    <div key={name}>
                      <div className="mb-1 text-xs text-muted-foreground">{name}</div>
                      <video
                        ref={(el) => {
                          videos.current[name] = el;
                        }}
                        src={`${baseUrl}/api/v1/controller/data/video?root=${encodeURIComponent(dataset)}&path=${encodeURIComponent(info.path)}`}
                        muted
                        playsInline
                        preload="auto"
                        onLoadedMetadata={(e) => {
                          e.currentTarget.currentTime = info.from_s + time;
                        }}
                        className="aspect-video w-full rounded bg-black"
                        data-testid={`video-${name}`}
                      />
                    </div>
                  ))}
                </div>
                <div className="mt-3 flex flex-wrap items-center gap-2">
                  <Button size="icon" variant="outline" className="h-8 w-8" onClick={() => setPlaying((p) => !p)} aria-label={playing ? t("controller.data.pause") : t("controller.data.play")} data-testid="data-play">
                    {playing ? <Pause className="h-4 w-4" /> : <Play className="h-4 w-4" />}
                  </Button>
                  <Button size="icon" variant="outline" className="h-8 w-8" onClick={() => stepFrame(-1)} aria-label={t("controller.data.previousFrame")}>
                    <ChevronLeft className="h-4 w-4" />
                  </Button>
                  <Button size="icon" variant="outline" className="h-8 w-8" onClick={() => stepFrame(1)} aria-label={t("controller.data.nextFrame")}>
                    <ChevronRight className="h-4 w-4" />
                  </Button>
                  <Select value={String(rate)} onValueChange={(value) => setRate(Number(value))}>
                    <SelectTrigger className="h-8 w-20">
                      <SelectValue />
                    </SelectTrigger>
                    <SelectContent>
                      {[0.25, 0.5, 1, 2, 4].map((value) => (
                        <SelectItem key={value} value={String(value)}>
                          {value}x
                        </SelectItem>
                      ))}
                    </SelectContent>
                  </Select>
                  <input
                    type="range"
                    aria-label={t("controller.data.timeline")}
                    min={0}
                    max={duration}
                    step={1 / data.fps}
                    value={time}
                    onChange={(e) => {
                      setPlaying(false);
                      seek(Number(e.target.value));
                    }}
                    className="min-w-40 flex-1 accent-primary"
                    data-testid="data-timeline"
                  />
                  <span className="text-xs tabular-nums text-muted-foreground" data-testid="data-time">
                    {time.toFixed(2)} / {duration.toFixed(2)} s · {frame}
                  </span>
                </div>
              </Section>

              <Section
                title={t("controller.data.chartTitle")}
                actions={
                  <Select value={String(joint)} onValueChange={(value) => setJoint(Number(value))}>
                    <SelectTrigger className="h-8 w-40">
                      <SelectValue />
                    </SelectTrigger>
                    <SelectContent>
                      {data.joints.map((name, index) => (
                        <SelectItem key={name} value={String(index)}>
                          {name}
                        </SelectItem>
                      ))}
                    </SelectContent>
                  </Select>
                }
              >
                <div className="h-56" data-testid="data-chart">
                  <ResponsiveContainer width="100%" height="100%">
                    <LineChart data={chart} margin={{ left: 0, right: 8, top: 4, bottom: 0 }}>
                      <CartesianGrid strokeDasharray="3 3" stroke="#3a4048" />
                      <XAxis dataKey="t" type="number" domain={[0, "dataMax"]} tickFormatter={(v: number) => v.toFixed(1)} fontSize={11} />
                      <YAxis fontSize={11} domain={["auto", "auto"]} width={44} tickFormatter={(v: number) => v.toFixed(2)} />
                      <Tooltip formatter={(v: number) => v.toFixed(3)} labelFormatter={(v: number) => `${v.toFixed(2)} s`} />
                      {row.control_intervals
                        .filter((i) => i.intervention)
                        .map((i) => (
                          <ReferenceArea key={i.start_s} x1={i.start_s} x2={i.end_s} fill="#ff5a5a" fillOpacity={0.15} />
                        ))}
                      {row.segments.map((s) => (
                        <ReferenceArea key={s.id} x1={s.start_s} x2={s.end_s} fill="#7ad17a" fillOpacity={0.12} />
                      ))}
                      <ReferenceLine x={time} stroke="#e6e6e6" />
                      <Line type="monotone" dataKey="state" name={t("controller.data.state")} stroke={SERIES} dot={false} strokeWidth={1.5} isAnimationActive={false} />
                      <Line type="monotone" dataKey="action" name={t("controller.data.action")} stroke={ACTION} dot={false} strokeWidth={1.5} isAnimationActive={false} />
                    </LineChart>
                  </ResponsiveContainer>
                </div>
                <div className="mt-1 flex flex-wrap gap-4 text-xs text-muted-foreground">
                  <span>
                    <span style={{ color: SERIES }}>━</span> {t("controller.data.state")}
                  </span>
                  <span>
                    <span style={{ color: ACTION }}>━</span> {t("controller.data.action")}
                  </span>
                  <span>{t("controller.data.intervalLegend")}</span>
                </div>
              </Section>

              <div className="grid gap-4 lg:grid-cols-2">
                <Section title={t("controller.data.annotateTitle")} hint={t("controller.data.annotateHint")}>
                  <div className="flex flex-wrap items-end gap-2">
                    <Field label={t("controller.data.result")}>
                      <Select value={row.result} onValueChange={(result) => annotate({ result })}>
                        <SelectTrigger className="h-8 w-28" data-testid="annotate-result">
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
                      <Input className="h-8" defaultValue={row.note} key={row.episode_index} onBlur={(e) => e.target.value !== row.note && annotate({ note: e.target.value })} />
                    </Field>
                    <Button size="sm" variant="outline" onClick={() => annotate({ excluded: !row.excluded })}>
                      {row.excluded ? t("controller.data.include") : t("controller.data.exclude")}
                    </Button>
                  </div>
                  <h4 className="mb-1 mt-4 text-xs font-medium">{t("controller.data.intervals")}</h4>
                  <ul className="space-y-0.5 text-xs tabular-nums">
                    {row.control_intervals.map((interval) => (
                      <li key={interval.start_s}>
                        <button className="hover:underline" onClick={() => seek(interval.start_s)}>
                          {interval.start_s.toFixed(2)}-{interval.end_s.toFixed(2)} s
                        </button>{" "}
                        {t(`controller.source.${interval.intervention ? "intervention" : (SOURCE_BY_ID[interval.source] ?? "none")}` as never)}
                      </li>
                    ))}
                  </ul>
                  {row.quality && (
                    <p className="mt-3 text-xs text-muted-foreground">
                      {t("controller.data.quality", {
                        rate: row.quality.followers[0]?.rate_hz ?? 0,
                        missing: row.quality.followers[0]?.missing ?? 0,
                        invalid: row.quality.invalid_frames,
                        late: Object.values(row.quality.cameras).reduce((sum, c) => sum + c.late_frames, 0),
                      })}
                    </p>
                  )}
                </Section>

                <Section title={t("controller.data.segmentsTitle")}>
                  <div className="grid grid-cols-2 gap-2 sm:grid-cols-3">
                    <Field label={t("controller.data.segStart")}>
                      <div className="flex gap-1">
                        <Input className="h-8" value={segStart} onChange={(e) => setSegStart(e.target.value)} inputMode="decimal" />
                        <Button size="sm" variant="ghost" className="h-8 px-1.5" onClick={() => setSegStart(time.toFixed(2))} title={t("controller.data.useCursor")}>
                          ⌖
                        </Button>
                      </div>
                    </Field>
                    <Field label={t("controller.data.segEnd")}>
                      <div className="flex gap-1">
                        <Input className="h-8" value={segEnd} onChange={(e) => setSegEnd(e.target.value)} inputMode="decimal" />
                        <Button size="sm" variant="ghost" className="h-8 px-1.5" onClick={() => setSegEnd(time.toFixed(2))} title={t("controller.data.useCursor")}>
                          ⌖
                        </Button>
                      </div>
                    </Field>
                    <Field label={t("controller.data.segKind")}>
                      <Select value={segKind} onValueChange={setSegKind}>
                        <SelectTrigger className="h-8">
                          <SelectValue />
                        </SelectTrigger>
                        <SelectContent>
                          <SelectItem value="phase">{t("controller.data.kindPhase")}</SelectItem>
                          <SelectItem value="correction">{t("controller.data.kindCorrection")}</SelectItem>
                          <SelectItem value="invalid">{t("controller.data.kindInvalid")}</SelectItem>
                        </SelectContent>
                      </Select>
                    </Field>
                    <Field label={t("controller.data.segLabel")}>
                      <Input className="h-8" value={segLabel} onChange={(e) => setSegLabel(e.target.value)} />
                    </Field>
                    <Field label={t("controller.data.note")} className="sm:col-span-2">
                      <Input className="h-8" value={segNote} onChange={(e) => setSegNote(e.target.value)} />
                    </Field>
                  </div>
                  <Button size="sm" variant="outline" className="mt-2" disabled={!segLabel || segStart === "" || segEnd === ""} onClick={addSegment}>
                    {t("controller.data.addSegment")}
                  </Button>
                  <ul className="mt-3 space-y-1 text-xs">
                    {row.segments.map((segment) => (
                      <li key={segment.id} className="flex items-center gap-2 tabular-nums">
                        <button className="hover:underline" onClick={() => seek(segment.start_s)}>
                          {segment.start_s.toFixed(2)}-{segment.end_s.toFixed(2)}
                        </button>
                        <Badge variant="secondary">{segment.label}</Badge>
                        <span className="truncate text-muted-foreground">{segment.note}</span>
                        <Button size="sm" variant="ghost" className="ml-auto h-6 px-2" onClick={() => removeSegment(segment.id)}>
                          {t("controller.teach.remove")}
                        </Button>
                      </li>
                    ))}
                  </ul>
                </Section>
              </div>
            </div>
          ) : (
            <EmptyState>{t("controller.data.chooseEpisode")}</EmptyState>
          )}
        </div>
      )}

      <Section title={t("controller.data.opsTitle")} hint={t("controller.data.opsHint")}>
        <div className="grid gap-4 md:grid-cols-3">
          <div className="space-y-2">
            <h4 className="text-xs font-medium">{t("controller.data.merge")}</h4>
            <Input className="h-8" placeholder={t("controller.data.mergeWith")} value={mergeWith.join(",")} onChange={(e) => setMergeWith(e.target.value.split(",").map((s) => s.trim()).filter(Boolean))} />
            <Input className="h-8" placeholder={t("controller.data.mergeOutput")} value={mergeOutput} onChange={(e) => setMergeOutput(e.target.value)} />
            <Button size="sm" variant="outline" disabled={!dataset || !mergeWith.length || !mergeOutput} onClick={() => quietly(merge())}>
              {t("controller.data.merge")}
            </Button>
          </div>
          <div className="space-y-2">
            <h4 className="text-xs font-medium">{t("controller.data.export")}</h4>
            <Input className="h-8" placeholder={t("controller.data.exportTo")} value={exportPath} onChange={(e) => setExportPath(e.target.value)} />
            <Button size="sm" variant="outline" disabled={!dataset || !exportPath} onClick={() => quietly(api("/data/export", { root: dataset, destination: exportPath }).then(() => toast.success(t("controller.data.done"))))}>
              {t("controller.data.export")}
            </Button>
          </div>
          <div className="space-y-2">
            <h4 className="text-xs font-medium">{t("controller.data.import")}</h4>
            <Input className="h-8" placeholder={t("controller.data.importFrom")} value={importPath} onChange={(e) => setImportPath(e.target.value)} />
            <Button size="sm" variant="outline" disabled={!importPath} onClick={() => quietly(api("/data/import", { source: importPath, base }).then(() => loadDatasets(base)))}>
              {t("controller.data.import")}
            </Button>
          </div>
        </div>
      </Section>
    </div>
  );
};

export default DataPanel;
