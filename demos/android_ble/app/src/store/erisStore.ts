import { create } from 'zustand';
import { DEFAULT_ENABLED, DEFAULT_STREAMS } from '../demoCommands';
import {
  BleErisLink,
  bluetoothState,
  requestBlePermissions,
  type ScanDevice,
  scanForDevices,
} from '../link/bleLink';
import { SimulatedErisLink } from '../link/simulatedLink';
import type { ErisLink, LinkListener, LinkState } from '../link/types';
import { TraceStore } from '../plot/traceStore';
import type { FeatureSchema } from '../protocol/schema';
import { ErisSession } from '../protocol/session';

export type Connection = 'idle' | 'connecting' | 'configuring' | 'ready';

/** `problem` explains why a stream the user ticked is not streaming. */
export interface StreamToggle {
  name: string;
  enabled: boolean;
  problem?: string;
}

export interface ConsoleLine {
  id: number;
  kind: 'sent' | 'text' | 'error' | 'app';
  text: string;
}

export interface LinkStats {
  kBytesPerSec: number;
  packetsPerSec: number;
  samplesPerSec: number;
  badFrames: number;
  undecodable: number;
}

interface ErisState {
  connection: Connection;
  deviceName: string | null;
  simulated: boolean;
  lastError: string | null;
  scanning: boolean;
  showAllDevices: boolean;
  devices: ScanDevice[];
  streams: StreamToggle[];
  streaming: boolean;
  /** Features currently decoded and plotted, in wire order. */
  live: FeatureSchema[];
  windowMs: number;
  console: ConsoleLine[];
  stats: LinkStats;
}

const SCAN_DURATION_MS = 15_000;
const DESC_TIMEOUT_MS = 3_000;
const SETTLE_MS = 150; // > one BLE streaming period (100 ms)
const CONSOLE_LINES = 200;
const STREAM_NAME = /^[A-Z0-9_]{1,16}$/;

export const useEris = create<ErisState>(() => ({
  connection: 'idle',
  deviceName: null,
  simulated: false,
  lastError: null,
  scanning: false,
  showAllDevices: false,
  devices: [],
  streams: DEFAULT_STREAMS.map((name) => ({ name, enabled: DEFAULT_ENABLED.has(name) })),
  streaming: true,
  live: [],
  windowMs: 5000,
  console: [],
  stats: { kBytesPerSec: 0, packetsPerSec: 0, samplesPerSec: 0, badFrames: 0, undecodable: 0 },
}));

const set = useEris.setState;
const get = useEris.getState;

/** Rolling sample history the charts read every frame (outside React state on purpose). */
export const traces = new TraceStore();

let consoleId = 0;
function log(kind: ConsoleLine['kind'], text: string) {
  set((s) => ({ console: [...s.console, { id: consoleId++, kind, text }].slice(-CONSOLE_LINES) }));
}

const session = new ErisSession({
  onData: (blocks) => traces.append(blocks),
  onText: (text) => log('text', text),
  onError: (text) => log('error', text.replace(/^<<ERROR>>/, '').replace(/<<ERROR>>$/, '')),
});

let link: ErisLink | null = null;
let statsTimer: ReturnType<typeof setInterval> | null = null;

const sleep = (ms: number) => new Promise((r) => setTimeout(r, ms));

/** Serialises stream reconfiguration so two quick taps cannot interleave S_F / DESC. */
let configChain: Promise<unknown> = Promise.resolve();
function withConfigLock<T>(fn: () => Promise<T>): Promise<T> {
  const run = configChain.then(fn, fn);
  configChain = run.catch(() => undefined);
  return run;
}

// ------------------------------------------------------------------ scanning

const found = new Map<string, ScanDevice>();
let stopScanFn: (() => void) | null = null;
let scanTimeout: ReturnType<typeof setTimeout> | null = null;

function publishDevices() {
  const { showAllDevices } = get();
  const devices = [...found.values()]
    .filter((d) => showAllDevices || d.hasUart || d.name?.startsWith('Eris'))
    .sort((a, b) => Number(b.hasUart) - Number(a.hasUart) || b.rssi - a.rssi);
  set({ devices });
}

export function setShowAllDevices(show: boolean) {
  set({ showAllDevices: show });
  publishDevices();
}

export async function startScan() {
  if (!(await requestBlePermissions())) {
    set({ lastError: 'Bluetooth permission is needed to find boards. The simulator works without it.' });
    return;
  }
  const state = await bluetoothState();
  if (state !== 'PoweredOn') {
    set({ lastError: state === 'Unsupported' ? 'This phone has no Bluetooth LE.' : 'Turn Bluetooth on to scan.' });
    return;
  }
  stopScan();
  found.clear();
  set({ scanning: true, devices: [], lastError: null });
  stopScanFn = scanForDevices(
    (d) => {
      found.set(d.id, d);
      publishDevices();
    },
    (message) => {
      stopScan();
      set({ lastError: `Scan failed: ${message}` });
    },
  );
  scanTimeout = setTimeout(stopScan, SCAN_DURATION_MS);
}

export function stopScan() {
  if (scanTimeout) clearTimeout(scanTimeout);
  scanTimeout = null;
  stopScanFn?.();
  stopScanFn = null;
  if (get().scanning) set({ scanning: false });
}

// ---------------------------------------------------------------- connection

const listener: LinkListener = {
  onBytes: (bytes) => session.feed(bytes),
  onState: (state: LinkState) => {
    switch (state.kind) {
      case 'connecting':
        set({ connection: 'connecting' });
        break;
      case 'ready':
        log('app', `Connected to ${get().deviceName} (MTU ${state.mtu})`);
        startStats();
        void (async () => {
          // The board may still be streaming for a previous host, or be in ASCII
          // mode (bootDefaults). Put it in a known state first.
          await send('S_OFF', false);
          await send('S_MODE BIN', false);
          await send('INFO', false);
          await applyStreams();
        })();
        break;
      case 'disconnected':
        link = null;
        if (statsTimer) clearInterval(statsTimer);
        statsTimer = null;
        set({ connection: 'idle', live: [], lastError: state.reason });
        break;
    }
  },
};

function beginLink(name: string, simulated: boolean) {
  link?.close();
  session.resetStream();
  session.setLayout([]);
  traces.clear();
  set({ deviceName: name, simulated, lastError: null, live: [], console: [] });
}

export function connect(device: ScanDevice) {
  stopScan();
  const name = device.name ?? device.id;
  beginLink(name, false);
  const ble = new BleErisLink(device.id, name, listener);
  link = ble;
  void ble.connect();
}

export function connectSimulator() {
  stopScan();
  beginLink('Eris simulator', true);
  const sim = new SimulatedErisLink(listener);
  link = sim;
  sim.connect();
}

export function disconnect() {
  link?.close();
}

// ------------------------------------------------------------------- streams

export function toggleStream(name: string) {
  set((s) => ({
    streams: s.streams.map((t) => (t.name === name ? { ...t, enabled: !t.enabled, problem: undefined } : t)),
  }));
  void applyStreams();
}

/** Add a stream name the firmware registers but the defaults don't list (e.g. IMU_0). */
export function addStream(raw: string): boolean {
  const name = raw.trim().toUpperCase();
  if (!STREAM_NAME.test(name) || get().streams.some((t) => t.name === name)) return false;
  set((s) => ({ streams: [...s.streams, { name, enabled: true }] }));
  void applyStreams();
  return true;
}

export function setStreaming(on: boolean) {
  set({ streaming: on });
  void withConfigLock(async () => {
    await send(on && get().live.length > 0 ? 'S_ON' : 'S_OFF');
  });
}

export const setWindow = (windowMs: number) => set({ windowMs });
export const clearPlots = () => traces.clear();
export const clearConsole = () => set({ console: [] });

/**
 * Changing the stream set is always the same sequence, whatever toggled it:
 *
 *     S_OFF          stop, so no packet of the old layout is mid-flight
 *     S_F a b c      select streams (an unknown name makes the firmware clear all)
 *     DESC           firmware reports each stream's byte layout
 *     S_ON           resume, now decodable
 */
function applyStreams(): Promise<void> {
  return withConfigLock(async () => {
    if (!link) return;
    const wanted = get()
      .streams.filter((t) => t.enabled)
      .map((t) => t.name);
    set({ connection: 'configuring' });

    try {
      await send('S_OFF', false);
      session.setLayout([]);
      await sleep(SETTLE_MS); // let a packet already on the radio arrive and be dropped

      let reply = await describe(wanted);
      if (!reply) return configFailed('No DESC reply. The firmware may predate the DESC command.');

      const problems = new Map<string, string>();
      if (!wanted.every((n) => reply!.has(n))) {
        // One unknown name makes S_F reject the whole set, so DESC comes back
        // empty. Probe names one at a time to find the culprit.
        for (const name of wanted) {
          const single = await describe([name]);
          if (!single?.has(name)) problems.set(name, "not in this firmware's streaming.cpp");
        }
      }
      for (const name of wanted) {
        if (!problems.has(name) && reply.has(name) && reply.get(name) === null) {
          problems.set(name, 'sample type has no ERIS_DESCRIBE block');
        }
      }

      const usable = wanted.filter((n) => !problems.has(n));
      if (usable.length !== wanted.length || !usable.every((n) => reply!.has(n))) {
        // The firmware must stream exactly the set we decode.
        reply = usable.length ? await describe(usable) : new Map();
        if (!reply) return configFailed('DESC stopped answering.');
      }
      const layout = usable.map((n) => reply!.get(n)).filter((s): s is FeatureSchema => !!s);

      problems.forEach((why, name) => log('error', `${name}: ${why}`));
      set((s) => ({
        streams: s.streams.map((t) => (problems.has(t.name) ? { ...t, enabled: false, problem: problems.get(t.name) } : t)),
        live: layout,
        connection: 'ready',
      }));
      session.setLayout(layout);
      traces.setLayout(layout);
      if (get().streaming && layout.length) await send('S_ON', false);
    } catch (e) {
      configFailed(`Configuring streams failed: ${(e as Error).message}`);
    }
  });
}

/**
 * Select `names` with S_F and return what DESC reports: feature -> schema
 * (null schema = undescribed). Null if DESC never finished.
 */
async function describe(names: string[]): Promise<Map<string, FeatureSchema | null> | null> {
  if (names.length === 0) {
    await send('S_F', false); // clears the set
    return new Map();
  }
  const pending = session.expectDescribe();
  await send(`S_F ${names.join(' ')}`, false);
  await send('DESC', false);
  const lines = await Promise.race([pending.promise, sleep(DESC_TIMEOUT_MS).then(() => null)]);
  if (!lines) {
    pending.cancel();
    return null;
  }
  return new Map(lines.map((l) => [l.feature, l.schema]));
}

function configFailed(message: string) {
  log('error', message);
  set({ connection: 'ready', live: [] });
}

// ------------------------------------------------------------------ commands

export function sendCommand(line: string) {
  const cmd = line.trim();
  if (cmd) void send(cmd);
}

async function send(line: string, echo = true): Promise<void> {
  const l = link;
  if (!l) return;
  if (echo) log('sent', line);
  try {
    await l.send(line);
  } catch (e) {
    log('error', `Send '${line}' failed: ${(e as Error).message}`);
  }
}

function startStats() {
  if (statsTimer) clearInterval(statsTimer);
  let last = session.stats();
  let lastAt = Date.now();
  statsTimer = setInterval(() => {
    const now = session.stats();
    const at = Date.now();
    const dt = Math.max(1, at - lastAt) / 1000;
    set({
      stats: {
        kBytesPerSec: (now.bytes - last.bytes) / 1000 / dt,
        packetsPerSec: (now.packets - last.packets) / dt,
        samplesPerSec: (now.samples - last.samples) / dt,
        badFrames: now.badFrames,
        undecodable: now.undecodable,
      },
    });
    last = now;
    lastAt = at;
  }, 1000);
}
