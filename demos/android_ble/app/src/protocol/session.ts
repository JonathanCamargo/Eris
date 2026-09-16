import { cobsDecode, FrameSplitter } from './cobs';
import { decodeData, type DescLine, type FeatureBlock, type FeatureSchema, parseDesc } from './schema';

type DescFeature = Extract<DescLine, { kind: 'feature' }>;

export interface SessionListener {
  /** A decoded D packet, already matched against the DESC layout. */
  onData(blocks: FeatureBlock[]): void;
  /** A TEXT ('T') packet that was not part of a DESC reply. */
  onText(text: string): void;
  /** An ERROR ('E') packet. */
  onError(text: string): void;
}

export interface SessionStats {
  bytes: number;
  packets: number;
  samples: number;
  /** COBS frames that failed to decode (link byte-loss). */
  badFrames: number;
  /** D packets that did not fit the layout (stale, or undescribed). */
  undecodable: number;
}

export interface PendingDescribe {
  promise: Promise<DescFeature[]>;
  cancel(): void;
}

const T = 'T'.charCodeAt(0);
const D = 'D'.charCodeAt(0);
const E = 'E'.charCodeAt(0);

function ascii(bytes: Uint8Array, from: number): string {
  let s = '';
  for (let i = from; i < bytes.length; i++) s += String.fromCharCode(bytes[i]);
  return s.trim();
}

/**
 * Turns the raw byte stream from an Eris link into meaning: console text, error
 * reports, DESC schemas and decoded samples.
 *
 * Transport-independent (BLE and the simulator feed it the same bytes) and free
 * of React Native imports, so it runs under plain jest.
 */
export class ErisSession {
  private readonly splitter = new FrameSplitter();
  private readonly counters: SessionStats = { bytes: 0, packets: 0, samples: 0, badFrames: 0, undecodable: 0 };

  /** Current decode layout, in S_F order. Empty = drop D packets. */
  private layout: FeatureSchema[] = [];

  private pending: DescFeature[] | null = null;
  private settle: ((features: DescFeature[]) => void) | null = null;

  constructor(private readonly listener: SessionListener) {}

  feed(bytes: Uint8Array): void {
    this.counters.bytes += bytes.length;
    this.splitter.feed(bytes, (frame) => this.handleFrame(frame));
  }

  private handleFrame(frame: Uint8Array): void {
    const packet = cobsDecode(frame);
    if (!packet || packet.length === 0) {
      this.counters.badFrames++;
      return;
    }
    this.counters.packets++;
    switch (packet[0]) {
      case D:
        this.handleData(packet.subarray(1));
        break;
      case T:
        this.handleText(ascii(packet, 1));
        break;
      case E:
        this.listener.onError(ascii(packet, 1));
        break;
      default:
        break; // 'C' / 'R' (classifier, regression) are not used here
    }
  }

  private handleData(body: Uint8Array): void {
    const blocks = this.layout.length ? decodeData(body, this.layout) : null;
    if (!blocks) {
      this.counters.undecodable++;
      return;
    }
    for (const b of blocks) this.counters.samples += b.samples.length;
    this.listener.onData(blocks);
  }

  private handleText(text: string): void {
    const desc = parseDesc(text);
    if (!desc || !this.pending) {
      // A DESC line nobody asked for (e.g. from a second host) is just text.
      if (text) this.listener.onText(text);
      return;
    }
    switch (desc.kind) {
      case 'feature':
        // DESC can race the streaming thread and repeat a feature; keep the first.
        if (!this.pending.some((f) => f.feature === desc.feature)) this.pending.push(desc);
        break;
      case 'malformed':
        this.listener.onError(`Bad DESC line: ${desc.reason}`);
        break;
      case 'end': {
        const done = this.pending;
        const settle = this.settle;
        this.pending = null;
        this.settle = null;
        settle?.(done);
        break;
      }
    }
  }

  /**
   * Arm the DESC collector. Call BEFORE sending `DESC`, then await the promise
   * with a timeout: firmware without DESC never answers.
   */
  expectDescribe(): PendingDescribe {
    this.cancelDescribe();
    this.pending = [];
    const promise = new Promise<DescFeature[]>((resolve) => {
      this.settle = resolve;
    });
    return { promise, cancel: () => this.cancelDescribe() };
  }

  cancelDescribe(): void {
    this.pending = null;
    this.settle = null;
  }

  setLayout(schemas: FeatureSchema[]): void {
    this.layout = schemas;
  }

  /** Drop any half-received frame, e.g. after a reconnect. */
  resetStream(): void {
    this.splitter.reset();
  }

  stats(): SessionStats {
    return { ...this.counters, badFrames: this.counters.badFrames + this.splitter.overflows };
  }
}
