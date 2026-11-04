/** The sync controller workspace: every controller function behind one run control bar. */
import React, { useEffect, useState } from "react";
import { Link, useSearchParams } from "react-router-dom";
import { useTranslation } from "react-i18next";
import { ArrowLeft } from "lucide-react";
import { Tabs, TabsContent, TabsList, TabsTrigger } from "@/components/ui/tabs";
import { ControllerProvider, useController } from "@/contexts/ControllerContext";
import CalibrationPanel from "@/components/controller/CalibrationPanel";
import DataPanel from "@/components/controller/DataPanel";
import DevicePanel from "@/components/controller/DevicePanel";
import PolicyPanel from "@/components/controller/PolicyPanel";
import RecordPanel from "@/components/controller/RecordPanel";
import ReplayPanel from "@/components/controller/ReplayPanel";
import RunBar from "@/components/controller/RunBar";
import TeachPanel from "@/components/controller/TeachPanel";
import TeleopPanel from "@/components/controller/TeleopPanel";

const EVENT_VISIBLE_MS = 15000;

const TABS = ["device", "calibration", "teleop", "teach", "policy", "replay", "record", "data"] as const;

const Workspace: React.FC = () => {
  const { t } = useTranslation();
  const { serviceError, events } = useController();
  const [params, setParams] = useSearchParams();
  const tab = TABS.includes(params.get("tab") as (typeof TABS)[number]) ? (params.get("tab") as string) : "device";
  const [now, setNow] = useState(() => Date.now());
  useEffect(() => {
    const timer = setInterval(() => setNow(Date.now()), 2000);
    return () => clearInterval(timer);
  }, []);
  // One line per kind, newest first, and only while it is recent.
  const seen = new Set<string>();
  const recent = events
    .filter((e) => (e.severity ?? 0) >= 1 && now - (e.received_at ?? 0) < EVENT_VISIBLE_MS)
    .reverse()
    .filter((e) => {
      const key = `${e.kind}:${e.arm ?? ""}`;
      if (seen.has(key)) return false;
      seen.add(key);
      return true;
    })
    .slice(0, 3);

  return (
    <div className="min-h-screen bg-background text-foreground">
      <header className="flex flex-wrap items-center gap-3 px-4 py-3 sm:px-6">
        <Link to="/" className="flex items-center gap-1 text-sm text-muted-foreground hover:text-foreground">
          <ArrowLeft className="h-4 w-4" />
          {t("controller.page.back")}
        </Link>
        <h1 className="text-base font-semibold">{t("controller.page.title")}</h1>
      </header>
      <RunBar />
      {serviceError && (
        <div className="bg-destructive/15 px-4 py-2 text-sm text-destructive sm:px-6" data-testid="service-error">
          {t("controller.page.serviceDown")}: {serviceError}
        </div>
      )}
      {recent.length > 0 && (
        <ul className="border-b bg-muted/40 px-4 py-1.5 text-xs sm:px-6" data-testid="event-strip">
          {recent.map((event) => (
            <li key={`${event.host_ts_us}-${event.kind}`} className={(event.severity ?? 0) >= 2 ? "text-destructive" : "text-warning"}>
              {t(`controller.event.${event.kind}` as never, { defaultValue: event.kind })}
              {event.arm ? ` · ${t(`controller.arm.${event.arm}` as never)}` : ""}
            </li>
          ))}
        </ul>
      )}
      <main className="px-4 py-4 sm:px-6">
        <Tabs value={tab} onValueChange={(value) => setParams({ tab: value }, { replace: true })}>
          <TabsList className="mb-4 flex h-auto flex-wrap justify-start">
            {TABS.map((name) => (
              <TabsTrigger key={name} value={name} data-testid={`tab-${name}`}>
                {t(`controller.tabs.${name}` as never)}
              </TabsTrigger>
            ))}
          </TabsList>
          <TabsContent value="device">
            <DevicePanel />
          </TabsContent>
          <TabsContent value="calibration">
            <CalibrationPanel />
          </TabsContent>
          <TabsContent value="teleop">
            <TeleopPanel />
          </TabsContent>
          <TabsContent value="teach">
            <TeachPanel />
          </TabsContent>
          <TabsContent value="policy">
            <PolicyPanel />
          </TabsContent>
          <TabsContent value="replay">
            <ReplayPanel />
          </TabsContent>
          <TabsContent value="record">
            <RecordPanel />
          </TabsContent>
          <TabsContent value="data">
            <DataPanel />
          </TabsContent>
        </Tabs>
      </main>
    </div>
  );
};

const Controller: React.FC = () => (
  <ControllerProvider>
    <Workspace />
  </ControllerProvider>
);

export default Controller;
