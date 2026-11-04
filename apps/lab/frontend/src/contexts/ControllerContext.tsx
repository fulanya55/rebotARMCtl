/**
 * Live view of the sync controller. Status and samples come from controller
 * feedback over one WebSocket; pages never show what they merely requested.
 */
import React, {
  createContext,
  useCallback,
  useContext,
  useEffect,
  useMemo,
  useRef,
  useState,
} from "react";
import { useApi } from "@/contexts/ApiContext";
import { toast } from "@/components/ui/sonner";

export const JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper"] as const;
export const ARMS = ["left_leader", "left_follower", "right_leader", "right_follower"] as const;
export type ArmName = (typeof ARMS)[number];

export interface ArmSample {
  arm: ArmName;
  sample_seq: number;
  position: number[];
  target: number[];
  velocity: number[];
  torque: number[];
  temperature: number[];
  mode: number;
  source: number;
  phase: number;
  valid_mask: number;
  flags: number;
  queue_depth: number;
  exec_seq: number;
  fault_mask: number;
}

export interface Binding {
  arm: ArmName;
  model: string;
  serial: string;
  dialect?: string;
}

export interface PortInfo {
  port: number;
  name: string;
  kind: "CAN" | "UART";
  binding: Binding | null;
  detected_model: string | null;
  online_mask: number;
  expected_mask: number;
  bus_load: number;
  tx_errors: number;
  rx_errors: number;
}

export interface ArmStatus {
  arm: ArmName;
  bound: boolean;
  binding: (Binding & { port: number }) | null;
  mode?: string;
  position?: number[];
  target?: number[];
  torque_on?: boolean;
  valid_mask?: number;
  fault_mask?: number;
  queue_depth?: number;
  source?: string;
  rate_hz?: number;
  missing?: number;
  invalid?: number;
  calibration_version?: number;
  serial?: string;
}

export interface PairStatus {
  pair: number;
  phase: "OFF" | "ALIGNING" | "ALIGNED" | "FOLLOWING" | "PAUSED";
  follow_mode: string;
  sync_period_us: number;
  sync_latency_us: number;
  rel_offset: number[];
}

export interface ControllerStatus {
  connected: boolean;
  uri: string | null;
  simulated: boolean;
  device: {
    firmware: string;
    protocol: number;
    uid: string;
    tick_hz: number;
    queue_points: number;
  } | null;
  generation: number;
  source: string;
  estop: "CLEAR" | "PRESSED" | "LATCHED" | null;
  host_alive: boolean;
  paused: boolean;
  uptime_s: number;
  overruns: number;
  max_tick_us: number;
  clock: { synced: boolean; best_rtt_us: number | null; drift_ppm: number; uncertainty_us: number | null } | null;
  arms: ArmStatus[];
  pairs: PairStatus[];
  ports: PortInfo[];
  execution: { job: number; paused: boolean; dt_s: number; arms: { arm: number; total: number; executed: number }[] } | null;
  teleop: { active: boolean; pairs: number[]; follow: string };
  guide: Record<string, boolean>;
  policy: {
    state: "idle" | "running" | "paused" | "suspended";
    token: number;
    buffered: number;
    process_alive: boolean;
    stats: {
      inferences?: number;
      last_inference_ms?: number | null;
      actions_sent?: number;
      rejected?: number;
      last_chunk_len?: number;
    };
  };
  intervention: {
    active: boolean;
    segments: { start_us: number; end_us: number; outcome: string }[];
  };
}

export interface ControllerEvent {
  origin: string;
  kind: string;
  host_ts_us: number;
  severity?: number;
  arm?: string | null;
  [key: string]: unknown;
}

export interface ModelSpec {
  model: number;
  key: string;
  label: string;
  family: string;
  port_kind: string;
  lower: number[];
  upper: number[];
  can_execute: boolean;
  can_hold: boolean;
  can_guide: boolean;
  home: number[];
  urdf: string;
  end_effector: string;
  notes: string;
}

interface ControllerContextValue {
  status: ControllerStatus | null;
  serviceError: string | null;
  events: ControllerEvent[];
  samplesRef: React.MutableRefObject<Record<string, ArmSample>>;
  models: ModelSpec[];
  /** POST (or GET when no body) under /api/v1/controller. Surfaces errors as toasts and rethrows. */
  api: <T = unknown>(path: string, body?: unknown, options?: { quiet?: boolean; method?: string }) => Promise<T>;
  modelOf: (arm: string) => ModelSpec | undefined;
}

const ControllerContext = createContext<ControllerContextValue | null>(null);

export const ControllerProvider: React.FC<{ children: React.ReactNode }> = ({ children }) => {
  const { baseUrl, wsBaseUrl } = useApi();
  const [status, setStatus] = useState<ControllerStatus | null>(null);
  const [serviceError, setServiceError] = useState<string | null>(null);
  const [events, setEvents] = useState<ControllerEvent[]>([]);
  const [models, setModels] = useState<ModelSpec[]>([]);
  const samplesRef = useRef<Record<string, ArmSample>>({});

  const api = useCallback(
    async <T,>(path: string, body?: unknown, options?: { quiet?: boolean; method?: string }): Promise<T> => {
      const method = options?.method ?? (body === undefined ? "GET" : "POST");
      const response = await fetch(`${baseUrl}/api/v1/controller${path}`, {
        method,
        headers: body === undefined ? undefined : { "Content-Type": "application/json" },
        body: body === undefined ? undefined : JSON.stringify(body),
      });
      if (!response.ok) {
        const payload = await response.json().catch(() => null);
        const message = payload?.detail?.message ?? payload?.detail ?? `HTTP ${response.status}`;
        const text = typeof message === "string" ? message : JSON.stringify(message);
        if (!options?.quiet) toast.error(text);
        const error = new Error(text) as Error & { code?: string };
        error.code = payload?.detail?.code;
        throw error;
      }
      return (await response.json()) as T;
    },
    [baseUrl],
  );

  useEffect(() => {
    let socket: WebSocket | null = null;
    let closed = false;
    let retry: ReturnType<typeof setTimeout> | undefined;

    const open = () => {
      socket = new WebSocket(`${wsBaseUrl}/api/v1/controller/ws`);
      socket.onmessage = (message) => {
        const data = JSON.parse(message.data);
        if (data.type === "error") {
          setServiceError(data.detail?.message ?? "service unavailable");
          return;
        }
        setServiceError(null);
        if (data.samples) samplesRef.current = data.samples;
        if (data.status) setStatus(data.status);
        if (data.events?.length) {
          setEvents((previous) => [...previous, ...data.events].slice(-200));
        }
      };
      socket.onclose = () => {
        if (!closed) retry = setTimeout(open, 1000);
      };
    };
    open();
    return () => {
      closed = true;
      if (retry) clearTimeout(retry);
      socket?.close();
    };
  }, [wsBaseUrl]);

  useEffect(() => {
    api<ModelSpec[]>("/models", undefined, { quiet: true })
      .then(setModels)
      .catch(() => undefined);
  }, [api, status?.connected]);

  const modelOf = useCallback(
    (arm: string) => {
      const key = status?.arms.find((a) => a.arm === arm)?.binding?.model;
      return models.find((m) => m.key === key);
    },
    [models, status],
  );

  const value = useMemo(
    () => ({ status, serviceError, events, samplesRef, models, api, modelOf }),
    [status, serviceError, events, models, api, modelOf],
  );
  return <ControllerContext.Provider value={value}>{children}</ControllerContext.Provider>;
};

export function useController(): ControllerContextValue {
  const value = useContext(ControllerContext);
  if (!value) throw new Error("useController needs a ControllerProvider");
  return value;
}

/** One arm's latest sample, re-rendering at most `hz` times a second. */
export function useArmSample(arm: string, hz = 10): ArmSample | undefined {
  const { samplesRef } = useController();
  const [sample, setSample] = useState<ArmSample | undefined>(samplesRef.current[arm]);
  useEffect(() => {
    const timer = setInterval(() => setSample(samplesRef.current[arm]), 1000 / hz);
    return () => clearInterval(timer);
  }, [arm, hz, samplesRef]);
  return sample;
}

/** Runs an action, leaving error display to `api`. Pass to onClick directly. */
export function quietly<T>(promise: Promise<T>): void {
  promise.catch(() => undefined);
}
