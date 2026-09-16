import { asciiBytes, base64ToBytes, bytesToBase64 } from '../src/protocol/base64';
import { cobsDecode, cobsEncode, FrameSplitter } from '../src/protocol/cobs';
import { decodeData, type FeatureSchema, parseDesc } from '../src/protocol/schema';
import { ErisSession } from '../src/protocol/session';
import { TraceStore } from '../src/plot/traceStore';

const bytes = (...v: number[]) => Uint8Array.from(v);
const concat = (...parts: Uint8Array[]) => {
  const out = new Uint8Array(parts.reduce((n, p) => n + p.length, 0));
  let o = 0;
  for (const p of parts) {
    out.set(p, o);
    o += p.length;
  }
  return out;
};

function schemaOf(line: string): FeatureSchema {
  const d = parseDesc(line);
  if (d?.kind !== 'feature' || !d.schema) throw new Error(`not a described feature: ${line}`);
  return d.schema;
}

/**
 * Golden vectors, generated with the Python `cobs` package the Eris driver uses:
 *   body = b'D' + struct.pack('<B4f4f', 2, 10,.5,-.25,1, 20,0,1,-1) + struct.pack('<B2f', 1, 15, 3.3)
 * i.e. a D packet for `S_F WAVES SINE`.
 */
const D_RAW = bytes(
  0x44, 0x02, 0x00, 0x00, 0x20, 0x41, 0x00, 0x00, 0x00, 0x3f, 0x00, 0x00, 0x80, 0xbe, 0x00, 0x00, 0x80, 0x3f,
  0x00, 0x00, 0xa0, 0x41, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3f, 0x00, 0x00, 0x80, 0xbf, 0x01, 0x00,
  0x00, 0x70, 0x41, 0x33, 0x33, 0x53, 0x40,
);
const D_COBS = bytes(
  0x03, 0x44, 0x02, 0x01, 0x03, 0x20, 0x41, 0x01, 0x01, 0x02, 0x3f, 0x01, 0x03, 0x80, 0xbe, 0x01, 0x03, 0x80,
  0x3f, 0x01, 0x03, 0xa0, 0x41, 0x01, 0x01, 0x01, 0x01, 0x01, 0x03, 0x80, 0x3f, 0x01, 0x04, 0x80, 0xbf, 0x01,
  0x01, 0x07, 0x70, 0x41, 0x33, 0x33, 0x53, 0x40,
);
const WAVES = schemaOf('DESC WAVES timestamp:f32:t,sine:f32,triangle:f32,square:f32');
const SINE = schemaOf('DESC SINE timestamp:f32:t,value:f32');

describe('COBS', () => {
  it('encodes the canonical example', () => {
    expect(cobsEncode(bytes(0x11, 0x22, 0x00, 0x33))).toEqual(bytes(0x03, 0x11, 0x22, 0x02, 0x33));
  });

  it('matches the Python-encoded data packet both ways', () => {
    expect(cobsDecode(D_COBS)).toEqual(D_RAW);
    expect(cobsEncode(D_RAW)).toEqual(D_COBS);
  });

  it('round-trips long runs and zeros like Python', () => {
    const run = Uint8Array.from({ length: 300 }, (_, i) => (i % 255) + 1);
    const long = concat(bytes(0), run, bytes(0, 0, 7));
    const enc = cobsEncode(long);
    expect(enc.length).toBe(306); // Python: ENC_LEN = 306
    expect(enc.slice(0, 4)).toEqual(bytes(0x01, 0xff, 0x01, 0x02));
    expect(cobsDecode(enc)).toEqual(long);
  });

  it('decodes exactly 254 non-zero bytes without a trailing zero', () => {
    const data = Uint8Array.from({ length: 254 }, (_, i) => i + 1);
    const enc = cobsEncode(data);
    expect(enc[0]).toBe(0xff);
    expect(cobsDecode(enc)).toEqual(data);
  });

  it('rejects a truncated frame', () => {
    expect(cobsDecode(bytes(0x05, 0x11, 0x22))).toBeNull();
  });

  it('reassembles frames across every chunk size', () => {
    const stream = concat(cobsEncode(D_RAW), bytes(0), cobsEncode(asciiBytes('THi')), bytes(0));
    for (let chunk = 1; chunk <= stream.length; chunk++) {
      const frames: Uint8Array[] = [];
      const splitter = new FrameSplitter();
      for (let i = 0; i < stream.length; i += chunk) splitter.feed(stream.subarray(i, i + chunk), (f) => frames.push(f));
      expect(frames).toHaveLength(2);
      expect(cobsDecode(frames[0])).toEqual(D_RAW);
    }
  });
});

describe('base64', () => {
  it('round-trips every remainder length', () => {
    for (let n = 0; n < 20; n++) {
      const data = Uint8Array.from({ length: n }, (_, i) => (i * 37) & 0xff);
      const b64 = bytesToBase64(data);
      expect(b64).toBe(Buffer.from(data).toString('base64'));
      expect(base64ToBytes(b64)).toEqual(data);
    }
  });
});

describe('DESC schema and data decoding', () => {
  it('parses DESC lines', () => {
    expect(WAVES.sampleSize).toBe(16);
    expect(WAVES.timeField?.name).toBe('timestamp');
    expect(WAVES.signals.map((f) => f.name)).toEqual(['sine', 'triangle', 'square']);
    expect(parseDesc('DESC END\r\n')).toEqual({ kind: 'end' });
    expect(parseDesc('Firmware: v3.0')).toBeNull();
  });

  it('sizes padding and meta fields but does not plot them', () => {
    // boolSample_t shape from ErisTapok2: u8 value + 3 pad bytes after a timestamp.
    const s = schemaOf('DESC GAIT timestamp:f32:t,value:u8,p0:pad:m,p1:pad:m,p2:pad:m');
    expect(s.sampleSize).toBe(8);
    expect(s.signals.map((f) => f.name)).toEqual(['value']);
  });

  it('handles UNDESCRIBED and malformed lines', () => {
    expect(parseDesc('DESC FSR UNDESCRIBED')).toEqual({ kind: 'feature', feature: 'FSR', schema: null });
    expect(parseDesc('DESC IMU ax:f64')?.kind).toBe('malformed');
  });

  it('decodes the golden data packet', () => {
    const blocks = decodeData(D_RAW.subarray(1), [WAVES, SINE])!;
    expect(blocks[0].samples).toHaveLength(2);
    expect(blocks[0].samples[0].timeMs).toBe(10);
    expect(blocks[0].samples[0].values).toEqual([0.5, -0.25, 1]);
    expect(blocks[0].samples[1].values).toEqual([0, 1, -1]);
    expect(blocks[1].samples[0].timeMs).toBe(15);
    expect(blocks[1].samples[0].values[0]).toBeCloseTo(3.3, 5);
  });

  it('rejects a packet for a different layout instead of misreading it', () => {
    expect(decodeData(D_RAW.subarray(1), [WAVES])).toBeNull();
    expect(decodeData(D_RAW.subarray(1), [SINE, WAVES])).toBeNull();
  });
});

describe('ErisSession', () => {
  const frame = (type: string, body: Uint8Array) => concat(cobsEncode(concat(asciiBytes(type), body)), bytes(0));
  const text = (s: string) => frame('T', asciiBytes(s));

  it('collects DESC, then decodes data', async () => {
    const data: unknown[] = [];
    const texts: string[] = [];
    const errors: string[] = [];
    const session = new ErisSession({
      onData: (b) => data.push(b),
      onText: (t) => texts.push(t),
      onError: (e) => errors.push(e),
    });

    const pending = session.expectDescribe();
    // DESC reply split across notifications, with unrelated text and a duplicate mixed in.
    const stream = concat(
      text('Firmware: v3.0'),
      text('DESC WAVES timestamp:f32:t,sine:f32,triangle:f32,square:f32'),
      text('DESC SINE timestamp:f32:t,value:f32'),
      text('DESC SINE timestamp:f32:t,value:f32'),
      text('DESC END'),
    );
    for (let i = 0; i < stream.length; i += 20) session.feed(stream.subarray(i, i + 20));

    const features = await pending.promise;
    expect(features.map((f) => f.feature)).toEqual(['WAVES', 'SINE']);
    expect(texts).toEqual(['Firmware: v3.0']);

    // Data before a layout is set is counted, not crashed on.
    session.feed(concat(cobsEncode(D_RAW), bytes(0)));
    expect(session.stats().undecodable).toBe(1);

    session.setLayout(features.map((f) => f.schema!));
    session.feed(concat(cobsEncode(D_RAW), bytes(0)));
    expect(data).toHaveLength(1);
    expect(session.stats().samples).toBe(3);

    session.feed(frame('E', asciiBytes('<<ERROR>>Unsupported command<<ERROR>>')));
    expect(errors).toEqual(['<<ERROR>>Unsupported command<<ERROR>>']);
  });
});

describe('TraceStore', () => {
  it('keeps the window, survives an unchanged layout, restarts on S_TIME', () => {
    const store = new TraceStore(64);
    store.setLayout([SINE]);
    store.append([{ schema: SINE, samples: Array.from({ length: 100 }, (_, i) => ({ timeMs: i * 10, values: [i] })) }]);

    const snap = store.snapshot('SINE', 200)!;
    expect(snap.newestMs).toBe(990);
    expect(snap.time[0]).toBeLessThanOrEqual(790); // enters from the left edge
    expect(snap.rateHz).toBeCloseTo(100, 0);

    store.setLayout([SINE, WAVES]); // toggling another stream on
    expect(store.snapshot('SINE', 200)).not.toBeNull();

    // S_TIME: timestamps step back -> the trace restarts.
    store.append([{ schema: SINE, samples: [{ timeMs: 5, values: [1] }] }]);
    expect(store.snapshot('SINE', 200)!.count).toBe(1);
  });
});
