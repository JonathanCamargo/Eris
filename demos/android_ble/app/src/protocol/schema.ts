/**
 * The firmware's self-described wire format (the DESC command).
 *
 * After `S_F WAVES SINE` the firmware answers `DESC` with one TEXT packet per
 * feature, in the same order the data packet concatenates them:
 *
 *     DESC WAVES timestamp:f32:t,sine:f32,triangle:f32,square:f32
 *     DESC SINE timestamp:f32:t,value:f32
 *     DESC END
 *
 * Mirrors eriscommon/src/eris_ascii.cpp (emitter) and
 * drivers/python/eris/eris.py `_parseDescBody` (the other host parser).
 */

export type FieldType = 'f32' | 'u8' | 'i8' | 'u16' | 'i16' | 'u32' | 'i32' | 'pad';
export type FieldRole = 'signal' | 'time' | 'meta';

export const FIELD_SIZE: Record<FieldType, number> = {
  f32: 4,
  u8: 1,
  i8: 1,
  u16: 2,
  i16: 2,
  u32: 4,
  i32: 4,
  pad: 1,
};

export interface Field {
  name: string;
  type: FieldType;
  role: FieldRole;
  offset: number;
}

export interface FeatureSchema {
  feature: string;
  fields: Field[];
  /** Bytes per sample -- DESC covers every byte of the struct, padding included. */
  sampleSize: number;
  timeField: Field | null;
  /** What gets plotted: plain signal fields, in struct order. */
  signals: Field[];
}

export type DescLine =
  /** `schema` is null when the firmware reported the type as UNDESCRIBED. */
  | { kind: 'feature'; feature: string; schema: FeatureSchema | null }
  | { kind: 'malformed'; text: string; reason: string }
  | { kind: 'end' };

function isFieldType(tag: string): tag is FieldType {
  return tag in FIELD_SIZE;
}

export function makeSchema(feature: string, fields: Field[]): FeatureSchema {
  return {
    feature,
    fields,
    sampleSize: fields.reduce((n, f) => n + FIELD_SIZE[f.type], 0),
    timeField: fields.find((f) => f.role === 'time' && f.type !== 'pad') ?? null,
    signals: fields.filter((f) => f.role === 'signal' && f.type !== 'pad'),
  };
}

/** Parse one TEXT packet. Returns null if it is not a DESC line at all. */
export function parseDesc(text: string): DescLine | null {
  const line = text.trim();
  if (!line.startsWith('DESC')) return null;
  if (line === 'DESC END') return { kind: 'end' };

  const first = line.indexOf(' ');
  const second = first < 0 ? -1 : line.indexOf(' ', first + 1);
  if (first < 0 || second < 0 || line.slice(0, first) !== 'DESC') {
    return { kind: 'malformed', text: line, reason: 'expected DESC <feature> <fields>' };
  }
  const feature = line.slice(first + 1, second);
  const body = line.slice(second + 1).trim();
  if (body === 'UNDESCRIBED') return { kind: 'feature', feature, schema: null };

  const fields: Field[] = [];
  let offset = 0;
  for (const spec of body.split(',')) {
    const bits = spec.split(':');
    if (bits.length < 2) return { kind: 'malformed', text: line, reason: `field '${spec}' has no type` };
    const tag = bits[1];
    if (!isFieldType(tag)) {
      return { kind: 'malformed', text: line, reason: `unknown type '${tag}' in '${spec}'` };
    }
    const role: FieldRole = bits[2] === 't' ? 'time' : bits[2] === 'm' || bits[2] === 'p' ? 'meta' : 'signal';
    fields.push({ name: bits[0], type: tag, role, offset });
    offset += FIELD_SIZE[tag];
  }
  return { kind: 'feature', feature, schema: makeSchema(feature, fields) };
}

/** One decoded sample: timestamp (ms since the firmware's t0, NaN if none) + signal values. */
export interface Sample {
  timeMs: number;
  values: number[];
}

export interface FeatureBlock {
  schema: FeatureSchema;
  samples: Sample[];
}

function readField(view: DataView, at: number, type: FieldType): number {
  switch (type) {
    case 'f32':
      return view.getFloat32(at, true);
    case 'u8':
      return view.getUint8(at);
    case 'i8':
      return view.getInt8(at);
    case 'u16':
      return view.getUint16(at, true);
    case 'i16':
      return view.getInt16(at, true);
    case 'u32':
      return view.getUint32(at, true);
    case 'i32':
      return view.getInt32(at, true);
    case 'pad':
      return 0;
  }
}

/**
 * Decode a D packet body (everything after the 'D' type byte).
 *
 * Layout, per feature in S_F order: `uint8 count` then `count` raw structs,
 * little-endian (see StreamSamples in eris_streaming.h).
 *
 * Returns null when the bytes do not fit `layout` exactly -- almost always a
 * packet that was already in flight when the stream set changed.
 */
export function decodeData(body: Uint8Array, layout: FeatureSchema[]): FeatureBlock[] | null {
  const view = new DataView(body.buffer, body.byteOffset, body.byteLength);
  const end = body.byteLength;
  let pos = 0;
  const blocks: FeatureBlock[] = [];

  for (const schema of layout) {
    if (pos >= end) return null;
    const count = view.getUint8(pos);
    pos++;
    if (pos + count * schema.sampleSize > end) return null;

    const samples: Sample[] = [];
    for (let s = 0; s < count; s++) {
      const timeMs = schema.timeField ? readField(view, pos + schema.timeField.offset, schema.timeField.type) : NaN;
      const values = schema.signals.map((f) => readField(view, pos + f.offset, f.type));
      samples.push({ timeMs, values });
      pos += schema.sampleSize;
    }
    blocks.push({ schema, samples });
  }
  return pos === end ? blocks : null;
}
