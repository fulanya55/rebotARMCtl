/** Unified run control (1.8): always visible, whatever page is open. */
import React from "react";
import { useTranslation } from "react-i18next";
import { Button } from "@/components/ui/button";
import { Badge } from "@/components/ui/badge";
import { Home, Pause, Play, RotateCcw, Square } from "lucide-react";
import { quietly, useController } from "@/contexts/ControllerContext";

const us = (value: number | undefined) => (value ? (value / 1000).toFixed(1) : "-");

const RunBar: React.FC = () => {
  const { t } = useTranslation();
  const { status, api } = useController();
  const connected = !!status?.connected;
  const estop = status?.estop ?? "CLEAR";
  const running = status?.source && status.source !== "none";
  const followingPair = status?.pairs.find((p) => p.phase !== "OFF");

  return (
    <div
      className={`flex flex-wrap items-center gap-3 border-b px-4 py-2 sm:px-6 ${
        estop !== "CLEAR" ? "bg-destructive/15" : "bg-card"
      }`}
      data-testid="run-bar"
    >
      <Badge variant={connected ? "default" : "secondary"}>
        {connected ? (status?.simulated ? t("controller.run.simulator") : t("controller.run.connected")) : t("controller.run.disconnected")}
      </Badge>
      <span className="text-sm">
        <span className="text-muted-foreground">{t("controller.run.source")}: </span>
        <span className="font-medium" data-testid="control-source">
          {t(`controller.source.${status?.source ?? "none"}` as never)}
        </span>
      </span>
      {followingPair && (
        <span className="text-sm tabular-nums text-muted-foreground">
          {t("controller.run.sync", { period: us(followingPair.sync_period_us), latency: us(followingPair.sync_latency_us) })}
        </span>
      )}
      {status?.clock?.synced && (
        <span className="hidden text-sm tabular-nums text-muted-foreground md:inline">
          {t("controller.run.clock", { value: ((status.clock.uncertainty_us ?? 0) / 1000).toFixed(2) })}
        </span>
      )}
      {estop !== "CLEAR" && (
        <span className="text-sm font-semibold text-destructive" data-testid="estop-banner">
          {estop === "PRESSED" ? t("controller.run.estopPressed") : t("controller.run.estopLatched")}
        </span>
      )}
      <div className="ml-auto flex flex-wrap items-center gap-2">
        {estop === "LATCHED" && (
          <Button size="sm" variant="outline" onClick={() => quietly(api("/run/reset-estop", {}))}>
            <RotateCcw className="mr-1 h-4 w-4" />
            {t("controller.run.resetEstop")}
          </Button>
        )}
        <Button size="sm" variant="outline" disabled={!connected || !running || status?.paused} onClick={() => quietly(api("/run/pause", {}))}>
          <Pause className="mr-1 h-4 w-4" />
          {t("controller.run.pause")}
        </Button>
        <Button size="sm" variant="outline" disabled={!connected || !status?.paused} onClick={() => quietly(api("/run/resume", {}))}>
          <Play className="mr-1 h-4 w-4" />
          {t("controller.run.resume")}
        </Button>
        <Button size="sm" variant="outline" disabled={!connected || !!running || estop !== "CLEAR"} onClick={() => quietly(api("/motion/home", {}))}>
          <Home className="mr-1 h-4 w-4" />
          {t("controller.run.home")}
        </Button>
        {/* Stop is never disabled while connected: there is no state in which it should wait. */}
        <Button size="sm" variant="destructive" disabled={!connected} onClick={() => quietly(api("/run/stop", { kind: "hold" }))} data-testid="stop-button">
          <Square className="mr-1 h-4 w-4" />
          {t("controller.run.stop")}
        </Button>
      </div>
    </div>
  );
};

export default RunBar;
