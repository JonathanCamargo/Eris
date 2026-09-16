import type { FeatureBlock, FeatureSchema } from '../protocol/schema';

/** Everything needed to draw one feature's chart. */
export interface Snapshot {
  labels: string[];
  /** ms since the firmware's t0, oldest first. */
  time: Float32Array;
  /** [signal][sample] */
  values: Float32Array[];
  count: number;
  newestMs: number;
  /** Measured sample rate over the window, from timestamps. */
  rateHz: number;
}

interface Traces {
  schema: FeatureSchema;
  time: Float32Array;
  values: Float32Array[];
  head: number; // next write index
  size: number;
}

function sameFields(a: FeatureSchema, b: FeatureSchema): boolean {
  return (
    a.fields.length === b.fields.length &&
    a.fields.every((f, i) => f.name === b.fields[i].name && f.type === b.fields[i].type && f.role === b.fields[i].role)
  );
}

/**
 * Rolling history of every plotted signal, in preallocated ring buffers.
 *
 * Decoded packets append here as they arrive; charts take a Snapshot of just the
 * visible window once per frame, so streaming never re-renders React.
 */
export class TraceStore {
  private features = new Map<string, Traces>();

  constructor(private readonly capacity = 4096) {}

  /**
   * Switch to a new stream set. History survives for any feature whose layout is
   * unchanged, so toggling one stream does not wipe the others.
   */
  setLayout(schemas: FeatureSchema[]): void {
    const next = new Map<string, Traces>();
    for (const schema of schemas) {
      const old = this.features.get(schema.feature);
      next.set(
        schema.feature,
        old && sameFields(old.schema, schema)
          ? old
          : {
              schema,
              time: new Float32Array(this.capacity),
              values: schema.signals.map(() => new Float32Array(this.capacity)),
              head: 0,
              size: 0,
            },
      );
    }
    this.features = next;
  }

  append(blocks: FeatureBlock[]): void {
    const cap = this.capacity;
    for (const block of blocks) {
      const tr = this.features.get(block.schema.feature);
      if (!tr) continue;
      for (const s of block.samples) {
        // Timestamps within a feature only ever increase, so a step back means
        // S_TIME reset t0 on the device: restart the trace rather than drawing a
        // line back to the left edge.
        if (tr.size > 0 && s.timeMs < tr.time[(tr.head - 1 + cap) % cap]) {
          tr.size = 0;
          tr.head = 0;
        }
        tr.time[tr.head] = s.timeMs;
        for (let i = 0; i < tr.values.length; i++) tr.values[i][tr.head] = s.values[i] ?? NaN;
        tr.head = (tr.head + 1) % cap;
        if (tr.size < cap) tr.size++;
      }
    }
  }

  clear(): void {
    for (const tr of this.features.values()) {
      tr.size = 0;
      tr.head = 0;
    }
  }

  /** Copy the newest `windowMs` of `feature`. Null if nothing received yet. */
  snapshot(feature: string, windowMs: number): Snapshot | null {
    const tr = this.features.get(feature);
    if (!tr || tr.size === 0) return null;
    const cap = this.capacity;
    const newestIdx = (tr.head - 1 + cap) % cap;
    const newest = tr.time[newestIdx];
    const from = newest - windowMs;

    // Walk back from the newest sample to find how many are visible, plus one
    // more so the line enters from the left edge.
    let n = 0;
    while (n < tr.size) {
      const t = tr.time[(newestIdx - n + cap) % cap];
      n++;
      if (t < from) break;
    }

    const start = (tr.head - n + cap) % cap;
    const time = new Float32Array(n);
    const values = tr.values.map(() => new Float32Array(n));
    for (let i = 0; i < n; i++) {
      const idx = (start + i) % cap;
      time[i] = tr.time[idx];
      for (let v = 0; v < values.length; v++) values[v][i] = tr.values[v][idx];
    }
    const span = n > 1 ? time[n - 1] - time[0] : 0;
    return {
      labels: tr.schema.signals.map((f) => f.name),
      time,
      values,
      count: n,
      newestMs: newest,
      rateHz: span > 0 ? ((n - 1) * 1000) / span : 0,
    };
  }
}
