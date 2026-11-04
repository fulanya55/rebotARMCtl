/** 1.3 Teleoperation: the controller runs the loop, this page configures and watches it. */
import React, { useCallback, useEffect, useState } from "react";
import { useTranslation } from "react-i18next";
import { Button } from "@/components/ui/button";
import { Input } from "@/components/ui/input";
import { Badge } from "@/components/ui/badge";
import { Switch } from "@/components/ui/switch";
import { Select, SelectContent, SelectItem, SelectTrigger, SelectValue } from "@/components/ui/select";
import { JOINTS, quietly, useController } from "@/contexts/ControllerContext";
import { EmptyState, Field, JointTable, Section, Stat } from "./common";

interface PairConfig {
  enabled: boolean;
  gripper_link: boolean;
  scale: number[];
  offset: number[];
  source_joint: number[];
  preset?: boolean;
  leader: { model: string; serial: string } | null;
  follower: { model: string; serial: string } | null;
}

const SIDES = [
  { index: 0, name: "left", leader: "left_leader", follower: "left_follower" },
  { index: 1, name: "right", leader: "right_leader", follower: "right_follower" },
] as const;

const TeleopPanel: React.FC = () => {
  const { t } = useTranslation();
  const { status, api } = useController();
  const [config, setConfig] = useState<Record<string, PairConfig>>({});
  const [speed, setSpeed] = useState("0.5");
  const [follow, setFollow] = useState("absolute");
  const [dirty, setDirty] = useState(false);

  const load = useCallback(() => {
    api<{ pairs: Record<string, PairConfig> }>("/teleop/config", undefined, { quiet: true })
      .then((r) => {
        setConfig(r.pairs);
        setDirty(false);
      })
      .catch(() => undefined);
  }, [api]);

  const bindings = status?.ports.map((p) => p.binding?.arm).join(",");
  useEffect(load, [load, bindings]);

  if (!status?.connected) return <EmptyState>{t("controller.common.connectFirst")}</EmptyState>;

  const active = status.teleop.active;
  const anyPair = Object.values(config).some((c) => c.leader && c.follower);
  const phases = status.pairs.map((p) => p.phase);
  const following = phases.includes("FOLLOWING");
  const aligning = phases.includes("ALIGNING");
  const aligned = phases.includes("ALIGNED");
  const paused = phases.includes("PAUSED");

  const edit = (index: number, change: Partial<PairConfig>) => {
    setConfig((c) => ({ ...c, [index]: { ...c[index], ...change } }));
    setDirty(true);
  };

  const editVector = (index: number, key: "scale" | "offset", joint: number, value: string) => {
    const vector = [...config[index][key]];
    vector[joint] = Number(value);
    edit(index, { [key]: vector });
  };

  const saveMapping = () =>
    quietly(
      api("/teleop/config", {
        pairs: Object.entries(config)
          .filter(([, c]) => c.leader && c.follower)
          .map(([index, c]) => ({
            pair: Number(index),
            enabled: c.enabled,
            gripper_link: c.gripper_link,
            scale: c.scale,
            offset: c.offset,
            source_joint: c.source_joint,
          })),
      }).then(() => setDirty(false)),
    );

  return (
    <div className="space-y-4">
      <Section title={t("controller.teleop.controlTitle")} hint={t("controller.teleop.controlHint")}>
        <div className="flex flex-wrap items-end gap-2">
          <Button size="sm" disabled={!anyPair || active || status.source !== "none"} onClick={() => quietly(api("/teleop/enter", {}))} data-testid="teleop-enter">
            {t("controller.teleop.enter")}
          </Button>
          <Field label={t("controller.teleop.alignSpeed")}>
            <Input className="h-8 w-24" value={speed} onChange={(e) => setSpeed(e.target.value)} inputMode="decimal" />
          </Field>
          <Button size="sm" variant="outline" disabled={!active || aligning || following} onClick={() => quietly(api("/teleop/align", { speed: Number(speed) }))} data-testid="teleop-align">
            {aligning ? t("controller.teleop.aligning") : t("controller.teleop.align")}
          </Button>
          <Field label={t("controller.teleop.followMode")}>
            <Select value={follow} onValueChange={setFollow}>
              <SelectTrigger className="h-8 w-36">
                <SelectValue />
              </SelectTrigger>
              <SelectContent>
                <SelectItem value="absolute">{t("controller.teleop.absolute")}</SelectItem>
                <SelectItem value="relative">{t("controller.teleop.relative")}</SelectItem>
              </SelectContent>
            </Select>
          </Field>
          <Button size="sm" variant="outline" disabled={!active || following || paused || (follow === "absolute" && !aligned)} onClick={() => quietly(api("/teleop/start", { follow }))} data-testid="teleop-start">
            {t("controller.teleop.start")}
          </Button>
          <Button size="sm" variant="outline" disabled={!following} onClick={() => quietly(api("/teleop/pause", {}))}>
            {t("controller.teleop.pause")}
          </Button>
          <Button size="sm" variant="outline" disabled={!paused} onClick={() => quietly(api("/teleop/resume", {}))}>
            {t("controller.teleop.resume")}
          </Button>
          <Button size="sm" variant="outline" disabled={!active} onClick={() => quietly(api("/teleop/end", {}))} data-testid="teleop-end">
            {t("controller.teleop.end")}
          </Button>
        </div>
        {follow === "absolute" && active && !aligned && !following && !paused && <p className="mt-2 text-xs text-muted-foreground">{t("controller.teleop.alignFirst")}</p>}
      </Section>

      <div className="grid gap-4 xl:grid-cols-2">
        {SIDES.map((side) => {
          const pair = config[side.index];
          const live = status.pairs[side.index];
          const leader = status.arms.find((a) => a.arm === side.leader);
          const follower = status.arms.find((a) => a.arm === side.follower);
          if (!leader?.bound && !follower?.bound) return null;
          return (
            <Section
              key={side.name}
              title={t(`controller.teleop.side_${side.name}` as never)}
              actions={live && <Badge data-testid={`pair-phase-${side.index}`}>{t(`controller.phase.${live.phase}` as never)}</Badge>}
            >
              <div className="mb-3 grid grid-cols-2 gap-3 sm:grid-cols-4">
                <Stat label={t("controller.teleop.period")} value={live?.sync_period_us ? `${(live.sync_period_us / 1000).toFixed(1)} ms` : "-"} />
                <Stat label={t("controller.teleop.latency")} value={live?.sync_latency_us ? `${(live.sync_latency_us / 1000).toFixed(1)} ms` : "-"} />
                <Stat label={t("controller.teleop.leaderRate")} value={leader?.rate_hz ? `${leader.rate_hz} Hz` : "-"} />
                <Stat label={t("controller.teleop.followerRate")} value={follower?.rate_hz ? `${follower.rate_hz} Hz` : "-"} />
              </div>
              <div className="grid gap-4 sm:grid-cols-2">
                <div>
                  <h4 className="mb-1 text-xs font-medium">{t(`controller.arm.${side.leader}` as never)}</h4>
                  <JointTable arm={side.leader} compact />
                </div>
                <div>
                  <h4 className="mb-1 text-xs font-medium">{t(`controller.arm.${side.follower}` as never)}</h4>
                  <JointTable arm={side.follower} compact />
                </div>
              </div>
              {live?.follow_mode === "RELATIVE" && live.phase !== "OFF" && (
                <p className="mt-2 text-xs tabular-nums text-muted-foreground">
                  {t("controller.teleop.relOffset")}: {live.rel_offset.slice(0, 6).map((v) => v.toFixed(2)).join(", ")}
                </p>
              )}
            </Section>
          );
        })}
      </div>

      <Section
        title={t("controller.teleop.mappingTitle")}
        hint={t("controller.teleop.mappingHint")}
        actions={
          <Button size="sm" variant="outline" disabled={!dirty || active} onClick={saveMapping}>
            {t("controller.teleop.saveMapping")}
          </Button>
        }
      >
        <div className="grid gap-6 xl:grid-cols-2">
          {SIDES.map((side) => {
            const pair = config[side.index];
            if (!pair?.leader || !pair?.follower) return null;
            return (
              <div key={side.name}>
                <div className="mb-2 flex flex-wrap items-center gap-3">
                  <span className="text-xs font-medium">{t(`controller.teleop.side_${side.name}` as never)}</span>
                  {pair.preset && <Badge variant="secondary">{t("controller.teleop.preset")}</Badge>}
                  <label className="flex items-center gap-1 text-xs">
                    <Switch checked={pair.enabled} disabled={active} onCheckedChange={(enabled) => edit(side.index, { enabled })} />
                    {t("controller.teleop.enabled")}
                  </label>
                  <label className="flex items-center gap-1 text-xs">
                    <Switch checked={pair.gripper_link} disabled={active} onCheckedChange={(gripper_link) => edit(side.index, { gripper_link })} />
                    {t("controller.teleop.gripperLink")}
                  </label>
                </div>
                <table className="w-full text-xs">
                  <thead className="text-muted-foreground">
                    <tr>
                      <th className="py-1 text-left font-normal">{t("controller.common.joint")}</th>
                      <th className="py-1 text-left font-normal">{t("controller.teleop.source")}</th>
                      <th className="py-1 text-left font-normal">{t("controller.teleop.scale")}</th>
                      <th className="py-1 text-left font-normal">{t("controller.teleop.offset")}</th>
                    </tr>
                  </thead>
                  <tbody>
                    {JOINTS.map((name, joint) => (
                      <tr key={name}>
                        <td className="py-0.5 pr-2">{name}</td>
                        <td className="py-0.5 pr-2">
                          <Select
                            value={String(pair.source_joint[joint])}
                            disabled={active}
                            onValueChange={(v) => {
                              const next = [...pair.source_joint];
                              next[joint] = Number(v);
                              edit(side.index, { source_joint: next });
                            }}
                          >
                            <SelectTrigger className="h-7 w-24">
                              <SelectValue />
                            </SelectTrigger>
                            <SelectContent>
                              {JOINTS.map((n, i) => (
                                <SelectItem key={n} value={String(i)}>
                                  {n}
                                </SelectItem>
                              ))}
                            </SelectContent>
                          </Select>
                        </td>
                        <td className="py-0.5 pr-2">
                          <Input className="h-7 w-20" disabled={active} value={pair.scale[joint]} onChange={(e) => editVector(side.index, "scale", joint, e.target.value)} />
                        </td>
                        <td className="py-0.5">
                          <Input className="h-7 w-20" disabled={active} value={pair.offset[joint]} onChange={(e) => editVector(side.index, "offset", joint, e.target.value)} />
                        </td>
                      </tr>
                    ))}
                  </tbody>
                </table>
              </div>
            );
          })}
        </div>
      </Section>
    </div>
  );
};

export default TeleopPanel;
