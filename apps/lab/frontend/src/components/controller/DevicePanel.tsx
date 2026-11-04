/** 1.1 Device connection: controller, port roles, cameras. Connecting only communicates. */
import React, { useEffect, useMemo, useState } from "react";
import { useTranslation } from "react-i18next";
import { Button } from "@/components/ui/button";
import { Input } from "@/components/ui/input";
import { Badge } from "@/components/ui/badge";
import { Select, SelectContent, SelectItem, SelectTrigger, SelectValue } from "@/components/ui/select";
import { ARMS, PortInfo, quietly, useController } from "@/contexts/ControllerContext";
import { useApi } from "@/contexts/ApiContext";
import { Field, Section, Stat } from "./common";

interface Candidate {
  uri: string;
  device: string;
  description: string;
  is_controller: boolean;
}

interface CameraConfig {
  source: string;
  width: number;
  height: number;
  fps: number;
}

const NONE = "__none__";
const SIM_URI = "sil://127.0.0.1:5760";

const DevicePanel: React.FC = () => {
  const { t } = useTranslation();
  const { status, api, models } = useController();
  const { baseUrl } = useApi();
  const [candidates, setCandidates] = useState<Candidate[]>([]);
  const [uri, setUri] = useState(SIM_URI);

  const refreshCandidates = () =>
    api<Candidate[]>("/candidates", undefined, { quiet: true })
      .then((found) => {
        setCandidates(found);
        const controller = found.find((c) => c.is_controller);
        if (controller) setUri(controller.uri);
      })
      .catch(() => undefined);

  useEffect(() => {
    void refreshCandidates();
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  const connected = !!status?.connected;

  return (
    <div className="grid gap-4 lg:grid-cols-2">
      <Section title={t("controller.device.controllerTitle")} hint={t("controller.device.controllerHint")}>
        <div className="flex flex-wrap items-end gap-2">
          <Field label={t("controller.device.address")} className="min-w-64 flex-1">
            <Input list="controller-candidates" value={uri} onChange={(e) => setUri(e.target.value)} disabled={connected} className="h-8" />
            <datalist id="controller-candidates">
              <option value={SIM_URI}>{t("controller.device.simulator")}</option>
              {candidates.map((c) => (
                <option key={c.uri} value={c.uri}>
                  {c.description}
                </option>
              ))}
            </datalist>
          </Field>
          {connected ? (
            <Button size="sm" variant="outline" onClick={() => quietly(api("/disconnect", {}))}>
              {t("controller.device.disconnect")}
            </Button>
          ) : (
            <Button size="sm" onClick={() => quietly(api("/connect", { uri }))} data-testid="connect-button">
              {t("controller.device.connect")}
            </Button>
          )}
          <Button size="sm" variant="outline" onClick={() => void refreshCandidates()} disabled={connected}>
            {t("controller.device.scan")}
          </Button>
        </div>
        {status?.device && (
          <div className="mt-4 grid grid-cols-2 gap-3 sm:grid-cols-4">
            <Stat label={t("controller.device.firmware")} value={status.device.firmware} />
            <Stat label={t("controller.device.protocol")} value={status.device.protocol} />
            <Stat label={t("controller.device.tick")} value={`${status.device.tick_hz} Hz`} />
            <Stat label={t("controller.device.maxTick")} value={`${status.max_tick_us} µs`} />
          </div>
        )}
      </Section>
      <RatesCard />
      <div className="lg:col-span-2">
        <PortsCard key={status?.uri ?? "none"} models={models.map((m) => ({ key: m.key, label: m.label, port_kind: m.port_kind, can_execute: m.can_execute }))} />
      </div>
      <div className="lg:col-span-2">
        <CamerasCard baseUrl={baseUrl} />
      </div>
    </div>
  );
};

const RatesCard: React.FC = () => {
  const { t } = useTranslation();
  const { status, api } = useController();
  const [canMs, setCanMs] = useState("4");
  const [starMs, setStarMs] = useState("8");
  const [upstreamMs, setUpstreamMs] = useState("4");
  const [idleMs, setIdleMs] = useState("10");
  const apply = () => {
    const can = Math.round(Number(canMs) * 1000);
    const star = Math.round(Number(starMs) * 1000);
    quietly(
      api("/rates", {
        motor_service_us: [can, can, star, star],
        state_read_us: [Math.round(Number(idleMs) * 1000), Math.round(Number(idleMs) * 1000), star, star],
        upstream_us: Math.round(Number(upstreamMs) * 1000),
      }),
    );
  };
  return (
    <Section title={t("controller.device.ratesTitle")} hint={t("controller.device.ratesHint")}>
      <div className="grid grid-cols-2 gap-3 sm:grid-cols-4">
        <Field label={t("controller.device.rateCan")}>
          <Input className="h-8" value={canMs} onChange={(e) => setCanMs(e.target.value)} inputMode="decimal" />
        </Field>
        <Field label={t("controller.device.rateStar")}>
          <Input className="h-8" value={starMs} onChange={(e) => setStarMs(e.target.value)} inputMode="decimal" />
        </Field>
        <Field label={t("controller.device.rateIdle")}>
          <Input className="h-8" value={idleMs} onChange={(e) => setIdleMs(e.target.value)} inputMode="decimal" />
        </Field>
        <Field label={t("controller.device.rateUpstream")}>
          <Input className="h-8" value={upstreamMs} onChange={(e) => setUpstreamMs(e.target.value)} inputMode="decimal" />
        </Field>
      </div>
      <Button size="sm" variant="outline" className="mt-3" onClick={apply} disabled={!status?.connected}>
        {t("controller.device.apply")}
      </Button>
    </Section>
  );
};

interface Row {
  arm: string;
  model: string;
  serial: string;
}

const PortsCard: React.FC<{ models: { key: string; label: string; port_kind: string; can_execute: boolean }[] }> = ({ models }) => {
  const { t } = useTranslation();
  const { status, api } = useController();
  const [rows, setRows] = useState<Record<number, Row>>({});
  const [busy, setBusy] = useState(false);

  // Follow the saved bindings until the user edits something.
  const ports = useMemo(() => status?.ports ?? [], [status?.ports]);
  useEffect(() => {
    setRows((previous) => {
      const next = { ...previous };
      for (const port of ports) {
        if (!next[port.port]) {
          next[port.port] = {
            arm: port.binding?.arm ?? NONE,
            model: port.binding?.model ?? NONE,
            serial: port.binding?.serial ?? "",
          };
        }
      }
      return next;
    });
  }, [ports]);

  const discover = async () => {
    setBusy(true);
    try {
      await api("/discover", {});
    } catch {
      /* shown by api */
    } finally {
      setBusy(false);
    }
  };

  const update = (port: number, change: Partial<Row>) => setRows((r) => ({ ...r, [port]: { ...r[port], ...change } }));

  const save = () =>
    quietly(
      api("/bind", {
        bindings: Object.entries(rows).map(([port, row]) => ({
          port: Number(port),
          arm: row.arm === NONE ? null : row.arm,
          model: row.model === NONE ? null : row.model,
          serial: row.serial || undefined,
        })),
      }),
    );

  return (
    <Section
      title={t("controller.device.portsTitle")}
      hint={t("controller.device.portsHint")}
      actions={
        <>
          <Button size="sm" variant="outline" onClick={() => void discover()} disabled={!status?.connected || busy} data-testid="discover-button">
            {t("controller.device.discover")}
          </Button>
          <Button size="sm" onClick={save} disabled={!status?.connected} data-testid="bind-button">
            {t("controller.device.saveBindings")}
          </Button>
        </>
      }
    >
      <div className="overflow-x-auto">
        <table className="w-full min-w-[720px] text-sm">
          <thead className="text-xs text-muted-foreground">
            <tr>
              <th className="py-1 text-left font-normal">{t("controller.device.port")}</th>
              <th className="py-1 text-left font-normal">{t("controller.device.detected")}</th>
              <th className="py-1 text-left font-normal">{t("controller.device.online")}</th>
              <th className="py-1 text-left font-normal">{t("controller.device.role")}</th>
              <th className="py-1 text-left font-normal">{t("controller.device.model")}</th>
              <th className="py-1 text-left font-normal">{t("controller.device.serial")}</th>
              <th className="py-1 text-right font-normal">{t("controller.device.load")}</th>
            </tr>
          </thead>
          <tbody>
            {ports.map((port: PortInfo) => {
              const row = rows[port.port];
              if (!row) return null;
              const choices = models.filter((m) => m.port_kind === port.kind);
              const online = port.online_mask.toString(2).replace(/0/g, "").length;
              return (
                <tr key={port.port} className="border-t" data-testid={`port-row-${port.port}`}>
                  <td className="py-2 pr-2 font-medium">{port.name}</td>
                  <td className="py-2 pr-2 text-xs">{port.detected_model ? t(`controller.detected.${port.detected_model}` as never) : "-"}</td>
                  <td className="py-2 pr-2">
                    <Badge variant={online === 7 ? "default" : online ? "destructive" : "secondary"}>{online}/7</Badge>
                  </td>
                  <td className="py-2 pr-2">
                    <Select value={row.arm} onValueChange={(arm) => update(port.port, { arm })}>
                      <SelectTrigger className="h-8 w-36">
                        <SelectValue />
                      </SelectTrigger>
                      <SelectContent>
                        <SelectItem value={NONE}>{t("controller.device.unused")}</SelectItem>
                        {ARMS.map((arm) => (
                          <SelectItem key={arm} value={arm}>
                            {t(`controller.arm.${arm}` as never)}
                          </SelectItem>
                        ))}
                      </SelectContent>
                    </Select>
                  </td>
                  <td className="py-2 pr-2">
                    <Select value={row.model} onValueChange={(model) => update(port.port, { model })}>
                      <SelectTrigger className="h-8 w-48">
                        <SelectValue />
                      </SelectTrigger>
                      <SelectContent>
                        <SelectItem value={NONE}>-</SelectItem>
                        {choices.map((m) => (
                          <SelectItem key={m.key} value={m.key} disabled={row.arm.endsWith("follower") && !m.can_execute}>
                            {m.label}
                          </SelectItem>
                        ))}
                      </SelectContent>
                    </Select>
                  </td>
                  <td className="py-2 pr-2">
                    <Input className="h-8 w-36" value={row.serial} placeholder={t("controller.device.serialHint")} onChange={(e) => update(port.port, { serial: e.target.value })} />
                  </td>
                  <td className="py-2 text-right text-xs tabular-nums">{port.bus_load.toFixed(1)}%</td>
                </tr>
              );
            })}
          </tbody>
        </table>
      </div>
    </Section>
  );
};

const CAMERAS_KEY = "rac.cameras";

const CamerasCard: React.FC<{ baseUrl: string }> = ({ baseUrl }) => {
  const { t } = useTranslation();
  const { api } = useController();
  const [running, setRunning] = useState(false);
  const [cameras, setCameras] = useState<Record<string, CameraConfig>>(() => {
    try {
      const saved = localStorage.getItem(CAMERAS_KEY);
      if (saved) return JSON.parse(saved);
    } catch {
      /* unavailable storage only loses a convenience */
    }
    return {
      env: { source: "sim", width: 640, height: 480, fps: 30 },
      wrist: { source: "sim", width: 640, height: 480, fps: 30 },
    };
  });
  const [stamp, setStamp] = useState(0);

  useEffect(() => {
    api<{ running: boolean; config: Record<string, CameraConfig> }>("/cameras", undefined, { quiet: true })
      .then((state) => {
        setRunning(state.running);
        if (state.running && Object.keys(state.config).length) setCameras(state.config);
      })
      .catch(() => undefined);
  }, [api]);

  const update = (name: string, change: Partial<CameraConfig>) => setCameras((c) => ({ ...c, [name]: { ...c[name], ...change } }));

  const start = async () => {
    try {
      localStorage.setItem(CAMERAS_KEY, JSON.stringify(cameras));
    } catch {
      /* see above */
    }
    const config: Record<string, unknown> = {};
    for (const [name, cam] of Object.entries(cameras)) {
      config[name] = { ...cam, source: /^\d+$/.test(cam.source) ? Number(cam.source) : cam.source };
    }
    await api("/cameras/start", { cameras: config });
    setRunning(true);
    // Capture needs a moment to create its shared memory before a preview can attach.
    setTimeout(() => setStamp(Date.now()), 800);
  };

  const stop = async () => {
    await api("/cameras/stop", {});
    setRunning(false);
  };

  return (
    <Section
      title={t("controller.device.camerasTitle")}
      hint={t("controller.device.camerasHint")}
      actions={
        running ? (
          <Button size="sm" variant="outline" onClick={() => quietly(stop())}>
            {t("controller.device.stopCameras")}
          </Button>
        ) : (
          <Button size="sm" onClick={() => quietly(start())} data-testid="start-cameras">
            {t("controller.device.startCameras")}
          </Button>
        )
      }
    >
      <div className="grid gap-4 md:grid-cols-2">
        {Object.entries(cameras).map(([name, camera]) => (
          <div key={name} className="space-y-2">
            <div className="flex flex-wrap items-end gap-2">
              <span className="w-12 pb-1.5 text-sm font-medium">{t(`controller.device.camera_${name}` as never, { defaultValue: name })}</span>
              <Field label={t("controller.device.cameraSource")}>
                <Input className="h-8 w-20" value={String(camera.source)} disabled={running} onChange={(e) => update(name, { source: e.target.value })} />
              </Field>
              <Field label={t("controller.device.cameraSize")}>
                <Select value={`${camera.width}x${camera.height}`} disabled={running} onValueChange={(v) => { const [w, h] = v.split("x").map(Number); update(name, { width: w, height: h }); }}>
                  <SelectTrigger className="h-8 w-28">
                    <SelectValue />
                  </SelectTrigger>
                  <SelectContent>
                    {["320x240", "640x480", "1280x720", "1920x1080"].map((size) => (
                      <SelectItem key={size} value={size}>
                        {size}
                      </SelectItem>
                    ))}
                  </SelectContent>
                </Select>
              </Field>
              <Field label="FPS">
                <Input className="h-8 w-16" type="number" value={camera.fps} disabled={running} onChange={(e) => update(name, { fps: Number(e.target.value) })} />
              </Field>
            </div>
            <div className="aspect-video overflow-hidden rounded border bg-black/60">
              {running && stamp > 0 ? (
                <img src={`${baseUrl}/api/v1/controller/cameras/${name}/stream?t=${stamp}`} alt={name} className="h-full w-full object-contain" />
              ) : (
                <div className="flex h-full items-center justify-center text-xs text-muted-foreground">{t("controller.device.noPreview")}</div>
              )}
            </div>
          </div>
        ))}
      </div>
    </Section>
  );
};

export default DevicePanel;
