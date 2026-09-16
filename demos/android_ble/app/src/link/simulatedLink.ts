import { asciiBytes } from '../protocol/base64';
import { cobsEncode } from '../protocol/cobs';
import type { ErisLink, LinkListener } from './types';

const MTU = 247;
const STREAM_PERIOD_MS = 100; // same period as the firmware on BLE
const TX_BUFFER_SIZE = 16;
const BUFFER_SIZE = 64;

interface SimFeature {
  name: string;
  rateHz: number;
  desc: string;
  generate: (tMs: number) => number[]; // signal values, struct order
  queue: Float32Array[]; // [timestamp, values...]
  emitted: number;
}

/**
 * A software stand-in for the ErisBLEDemo firmware.
 *
 * It speaks the real wire protocol -- COBS-framed packets chopped into MTU-sized
 * "notifications" -- so everything above the transport (framing, DESC, decoding,
 * plotting) runs exactly as it does against a board. Use it on an emulator or to
 * try the app before flashing anything.
 *
 * Mirrors demos/android_ble/firmware/ErisBLEDemo (streams + demo commands) and
 * eriscommon serialcom.cpp (universal commands, error packets).
 */
export class SimulatedErisLink implements ErisLink {
  readonly name = 'Eris simulator';

  private freqHz = 1;
  private amp = 1;
  private rgb = 'OFF';
  private wavesPhase = 0;
  private sineIdx = 0;
  private selected: SimFeature[] = [];
  private streaming = false;
  private readonly bootMs = Date.now();
  private t0Ms = Date.now();
  private timer: ReturnType<typeof setInterval> | null = null;
  private seed = 42;

  private readonly features: SimFeature[] = [
    this.feature('WAVES', 100, 'timestamp:f32:t,sine:f32,triangle:f32,square:f32', () => {
      this.wavesPhase = (this.wavesPhase + this.freqHz / 100) % 1;
      const p = this.wavesPhase;
      return [this.amp * Math.sin(2 * Math.PI * p), this.amp * (4 * Math.abs(p - 0.5) - 1), p < 0.5 ? this.amp : -this.amp];
    }),
    this.feature('ANALOG', 50, 'timestamp:f32:t,a0:f32,a1:f32', (t) => {
      const s = t / 1000;
      return [
        1.2 + 0.25 * Math.sin(2 * Math.PI * 0.07 * s) + this.noise(0.03),
        1.65 + 0.9 * Math.sin(2 * Math.PI * 0.25 * s) + this.noise(0.02),
      ];
    }),
    this.feature('TEMP', 10, 'timestamp:f32:t,value:f32', (t) => [
      24.5 + 1.5 * Math.sin((2 * Math.PI * t) / 60000) + this.noise(0.25),
    ]),
    // Same formula as eriscommon's SineWave module: 3.3 V, 10000-sample cycle.
    this.feature('SINE', 100, 'timestamp:f32:t,value:f32', () => {
      this.sineIdx = this.sineIdx >= 10000 ? 0 : this.sineIdx + 1;
      return [3.3 * Math.sin((2 * Math.PI * this.sineIdx) / 1e4)];
    }),
  ];

  constructor(private readonly listener: LinkListener) {}

  private feature(name: string, rateHz: number, desc: string, generate: (tMs: number) => number[]): SimFeature {
    return { name, rateHz, desc, generate, queue: [], emitted: 0 };
  }

  /** Small deterministic PRNG so the simulator looks the same every run. */
  private noise(scale: number): number {
    this.seed = (this.seed * 1664525 + 1013904223) % 4294967296;
    return (this.seed / 4294967296 - 0.5) * 2 * scale;
  }

  connect(): void {
    this.listener.onState({ kind: 'connecting' });
    setTimeout(() => {
      if (this.timer !== null || this.closed) return;
      this.listener.onState({ kind: 'ready', mtu: MTU });
      this.timer = setInterval(() => this.tick(), STREAM_PERIOD_MS);
    }, 400);
  }

  private closed = false;

  /** Sensor threads + streaming thread, one pass. */
  private tick(): void {
    const sinceBootS = (Date.now() - this.bootMs) / 1000;
    const nowMs = Date.now() - this.t0Ms;
    for (const f of this.features) {
      const due = Math.floor(sinceBootS * f.rateHz);
      while (f.emitted < due) {
        f.emitted++;
        const t = nowMs - ((due - f.emitted) * 1000) / f.rateHz;
        f.queue.push(Float32Array.from([t, ...f.generate(t)]));
        if (f.queue.length > BUFFER_SIZE) f.queue.shift(); // ErisBuffer overwrite
      }
    }
    if (!this.streaming) return;

    const parts: number[] = ['D'.charCodeAt(0)];
    for (const f of this.selected) {
      const n = Math.min(f.queue.length, TX_BUFFER_SIZE);
      parts.push(n);
      for (let i = 0; i < n; i++) {
        const sample = f.queue.shift()!;
        const raw = new Uint8Array(sample.buffer, sample.byteOffset, sample.byteLength); // little-endian on every phone
        for (const b of raw) parts.push(b);
      }
    }
    this.emit(Uint8Array.from(parts));
  }

  async send(line: string): Promise<void> {
    await new Promise((r) => setTimeout(r, 8)); // a connection event or two
    const tokens = line.trim().split(/\s+/).filter(Boolean);
    if (tokens.length === 0) return;
    const [cmd, ...args] = tokens;
    const text = (s: string) => this.emit(Uint8Array.from([...asciiBytes('T'), ...asciiBytes(s)]));
    const error = (s: string) => this.emit(Uint8Array.from([...asciiBytes('E'), ...asciiBytes(s)]));
    const num = (s: string | undefined) => (s === undefined || Number.isNaN(Number(s)) ? null : Number(s));
    const clamp = (v: number, lo: number, hi: number) => Math.min(hi, Math.max(lo, v));

    switch (cmd) {
      case 'INFO':
        text('Firmware: "v3.0" Eris BLE demo (simulated)');
        break;
      case 'S_F': {
        const picked = args.map((a) => this.features.find((f) => f.name === a));
        if (picked.some((f) => !f)) {
          this.selected = [];
          error('StreamingSetFeatures');
          error('<<ERROR>>Unsupported command<<ERROR>>');
        } else {
          this.selected = picked as SimFeature[];
        }
        break;
      }
      case 'S_ON':
        this.streaming = true;
        break;
      case 'S_OFF':
        this.streaming = false;
        break;
      case 'S_MODE':
        if (args[0]?.toUpperCase().startsWith('ASCII')) text('The simulator only speaks S_MODE BIN');
        break;
      case 'DESC':
        this.selected.forEach((f) => text(`DESC ${f.name} ${f.desc}`));
        text('DESC END');
        break;
      case 'S_TIME':
      case 'TIME0':
        this.t0Ms = Date.now();
        break;
      case 'ON':
      case 'OFF':
        break; // firmware answers on USB Serial only
      case 'FREQ': {
        const v = num(args[0]);
        if (v !== null) this.freqHz = clamp(v, 0.1, 20);
        text(`FREQ ${this.freqHz.toFixed(2)} Hz`);
        break;
      }
      case 'AMP': {
        const v = num(args[0]);
        if (v !== null) this.amp = clamp(v, 0, 10);
        text(`AMP ${this.amp.toFixed(2)}`);
        break;
      }
      case 'RGB': {
        const a = args[0]?.toUpperCase();
        if (!a) {
          text('RGB expects R, G, B, W or OFF');
          break;
        }
        this.rgb = ['R', 'G', 'B', 'W'].includes(a[0]) ? a[0] : 'OFF';
        text(`RGB ${this.rgb}`);
        break;
      }
      case 'PING':
        text(`PONG ${Date.now() - this.bootMs} ms`);
        break;
      case 'STATUS':
        text(
          `STATUS freq=${this.freqHz.toFixed(2)} amp=${this.amp.toFixed(2)} rgb=${this.rgb} up=${Math.floor(
            (Date.now() - this.bootMs) / 1000,
          )}s`,
        );
        break;
      default:
        error('<<ERROR>>Unsupported command<<ERROR>>');
    }
  }

  /** COBS-frame a packet and deliver it split at the MTU, like the radio. */
  private emit(packet: Uint8Array): void {
    if (this.closed) return;
    const encoded = cobsEncode(packet);
    const frame = new Uint8Array(encoded.length + 1);
    frame.set(encoded);
    const chunk = MTU - 3;
    for (let i = 0; i < frame.length; i += chunk) this.listener.onBytes(frame.slice(i, i + chunk));
  }

  close(): void {
    if (this.closed) return;
    this.closed = true;
    if (this.timer !== null) clearInterval(this.timer);
    this.timer = null;
    this.listener.onState({ kind: 'disconnected', reason: null });
  }
}
