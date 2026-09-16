import { Canvas, Path, Skia } from '@shopify/react-native-skia';
import { Card, chart, colors } from '@robiolab/native-ui';
import * as React from 'react';
import { type LayoutChangeEvent, Pressable, Text, View } from 'react-native';
import { Gesture, GestureDetector } from 'react-native-gesture-handler';
import type { FeatureSchema } from '../protocol/schema';
import { traces } from '../store/erisStore';
import type { Snapshot } from './traceStore';

const HEIGHT = 180;
const LEFT = 44; // room for y labels
const RIGHT = 6;
const TOP = 8;
const BOTTOM = 18; // room for x labels
const FRAME_MS = 33; // ~30 fps is smooth for a scrolling trace
const LEGEND_MS = 200; // numbers change at a readable 5 Hz

/**
 * Charts for one feature. The brand chart palette has three validated slots, so
 * a feature with more signals is split into small multiples of up to three
 * rather than inventing extra colors (IMU: ax/ay/az, then wx/wy/wz).
 */
export function FeatureCharts({ schema, windowMs }: { schema: FeatureSchema; windowMs: number }) {
  const per = chart.series.length;
  const groups: number[][] = [];
  for (let i = 0; i < schema.signals.length; i += per) {
    groups.push(schema.signals.slice(i, i + per).map((_, k) => i + k));
  }
  return (
    <>
      {groups.map((signals, g) => (
        <LiveChart
          key={`${schema.feature}-${g}`}
          feature={schema.feature}
          title={groups.length > 1 ? `${schema.feature}  (${g + 1}/${groups.length})` : schema.feature}
          signals={signals}
          labels={signals.map((i) => schema.signals[i].name)}
          windowMs={windowMs}
        />
      ))}
    </>
  );
}

interface LiveChartProps {
  feature: string;
  title: string;
  /** Indices into the feature's signals that this chart draws. */
  signals: number[];
  labels: string[];
  windowMs: number;
}

/** Y range that grows at once (nothing clips) and shrinks gently (no breathing). */
function easeScale(prev: { lo: number; hi: number } | null, lo: number, hi: number) {
  if (!prev) return { lo, hi };
  return {
    lo: lo < prev.lo ? lo : prev.lo + (lo - prev.lo) * 0.08,
    hi: hi > prev.hi ? hi : prev.hi + (hi - prev.hi) * 0.08,
  };
}

function niceStep(range: number, count: number): number {
  const raw = range / count;
  const mag = 10 ** Math.floor(Math.log10(raw));
  const norm = raw / mag;
  return (norm < 1.5 ? 1 : norm < 3 ? 2 : norm < 7 ? 5 : 10) * mag;
}

function formatTick(v: number, step: number): string {
  const decimals = Math.min(4, Math.max(0, -Math.floor(Math.log10(step))));
  return (Math.abs(v) < step / 1000 ? 0 : v).toFixed(decimals); // avoid "-0.0"
}

export function formatValue(v: number): string {
  if (!Number.isFinite(v)) return '–';
  const a = Math.abs(v);
  return a >= 1000 ? v.toFixed(0) : a >= 10 ? v.toFixed(1) : v.toFixed(3);
}

function nearestIndex(time: Float32Array, count: number, t: number): number {
  let lo = 0;
  let hi = count - 1;
  while (lo < hi) {
    const mid = (lo + hi) >> 1;
    if (time[mid] < t) lo = mid + 1;
    else hi = mid;
  }
  return lo > 0 && Math.abs(time[lo - 1] - t) < Math.abs(time[lo] - t) ? lo - 1 : lo;
}

export function LiveChart({ feature, title, signals, labels, windowMs }: LiveChartProps) {
  const [width, setWidth] = React.useState(0);
  const [, setFrame] = React.useState(0);
  const [hidden, setHidden] = React.useState<Set<number>>(new Set());
  const [inspectX, setInspectX] = React.useState<number | null>(null);
  const frozen = React.useRef<Snapshot | null>(null);
  const scale = React.useRef<{ lo: number; hi: number } | null>(null);
  const legend = React.useRef<{ at: number; values: number[]; rate: number }>({ at: 0, values: [], rate: 0 });

  // Redraw loop. Only this chart re-renders; the rest of the screen is untouched.
  React.useEffect(() => {
    let raf = 0;
    let last = 0;
    const loop = (now: number) => {
      if (now - last >= FRAME_MS && frozen.current === null) {
        last = now;
        setFrame((f) => (f + 1) % 1_000_000);
      }
      raf = requestAnimationFrame(loop);
    };
    raf = requestAnimationFrame(loop);
    return () => cancelAnimationFrame(raf);
  }, []);

  const snap = frozen.current ?? traces.snapshot(feature, windowMs);
  const plotW = Math.max(1, width - LEFT - RIGHT);
  const plotH = HEIGHT - TOP - BOTTOM;
  const bottom = TOP + plotH;

  // ---- scale over the visible, non-hidden signals ----
  let lo = Infinity;
  let hi = -Infinity;
  if (snap) {
    signals.forEach((sig, slot) => {
      if (hidden.has(slot)) return;
      const vs = snap.values[sig];
      for (let i = 0; i < snap.count; i++) {
        const v = vs[i];
        if (Number.isFinite(v)) {
          if (v < lo) lo = v;
          if (v > hi) hi = v;
        }
      }
    });
  }
  if (!Number.isFinite(lo)) {
    lo = -1;
    hi = 1;
  }
  if (hi - lo < 1e-6) {
    lo -= 0.5;
    hi += 0.5;
  }
  const pad = (hi - lo) * 0.08;
  if (frozen.current === null) scale.current = easeScale(scale.current, lo - pad, hi + pad);
  const yLo = scale.current?.lo ?? lo;
  const yHi = scale.current?.hi ?? hi;
  const y = (v: number) => bottom - ((v - yLo) / (yHi - yLo)) * plotH;
  const step = niceStep(yHi - yLo, 4);
  const ticks: number[] = [];
  for (let t = Math.ceil(yLo / step) * step; t <= yHi; t += step) ticks.push(t);

  const tEnd = snap?.newestMs ?? 0;
  const tStart = tEnd - windowMs;
  const x = (t: number) => LEFT + ((t - tStart) / windowMs) * plotW;

  // ---- one path per series ----
  const paths = signals.map((sig, slot) => {
    const p = Skia.Path.Make();
    if (!snap || hidden.has(slot) || width === 0) return p;
    let pen = false;
    const vs = snap.values[sig];
    for (let i = 0; i < snap.count; i++) {
      const v = vs[i];
      if (!Number.isFinite(v)) {
        pen = false;
        continue;
      }
      const px = x(snap.time[i]);
      const py = y(v);
      if (pen) p.lineTo(px, py);
      else p.moveTo(px, py);
      pen = true;
    }
    return p;
  });

  // ---- legend numbers at 5 Hz ----
  const nowMs = Date.now();
  if (snap && frozen.current === null && nowMs - legend.current.at >= LEGEND_MS) {
    legend.current = {
      at: nowMs,
      values: signals.map((sig) => snap.values[sig][snap.count - 1]),
      rate: snap.rateHz,
    };
  }

  // ---- long-press and drag to inspect (a plain drag still scrolls the page) ----
  // Memoized: this component re-renders ~30x/s, and a new gesture object each
  // render would make the gesture handler restart and drop an active drag.
  const gesture = React.useMemo(
    () =>
      Gesture.Pan()
        .activateAfterLongPress(250)
        .runOnJS(true)
        .onStart((e) => {
          frozen.current = traces.snapshot(feature, windowMs);
          setInspectX(e.x);
        })
        .onUpdate((e) => setInspectX(e.x))
        .onFinalize(() => {
          frozen.current = null;
          setInspectX(null);
        }),
    [feature, windowMs],
  );

  let inspect: React.ReactNode = null;
  if (inspectX !== null && snap && snap.count > 0) {
    const tAt = tStart + ((Math.min(Math.max(inspectX, LEFT), LEFT + plotW) - LEFT) / plotW) * windowMs;
    const idx = nearestIndex(snap.time, snap.count, tAt);
    const cx = x(snap.time[idx]);
    const rows = signals
      .map((sig, slot) => ({ slot, label: labels[slot], v: snap.values[sig][idx] }))
      .filter((r) => !hidden.has(r.slot) && Number.isFinite(r.v));
    const boxLeft = cx + 12 + 140 < LEFT + plotW ? cx + 12 : cx - 12 - 140;
    inspect = (
      <>
        <View style={{ position: 'absolute', left: cx, top: TOP, width: 1, height: plotH, backgroundColor: colors.muted }} />
        {rows.map((r) => (
          <View
            key={r.slot}
            style={{
              position: 'absolute',
              left: cx - 6,
              top: y(r.v) - 6,
              width: 12,
              height: 12,
              borderRadius: 6,
              borderWidth: 2, // surface ring keeps the dot legible over the lines
              borderColor: colors.background,
              backgroundColor: chart.series[r.slot],
            }}
          />
        ))}
        <View
          className="rounded-brand bg-background px-3 py-2 shadow-brand-raised"
          style={{ position: 'absolute', left: boxLeft, top: TOP + 4, width: 140 }}
        >
          <Text className="text-xs text-muted" style={{ fontVariant: ['tabular-nums'] }}>
            t = {snap.time[idx].toFixed(1)} ms
          </Text>
          {rows.map((r) => (
            <View key={r.slot} className="mt-1 flex-row items-center">
              <View style={{ width: 12, height: 3, borderRadius: 2, backgroundColor: chart.series[r.slot] }} />
              <Text className="ml-2 flex-1 text-xs text-primary">{r.label}</Text>
              <Text className="text-xs text-primary" style={{ fontVariant: ['tabular-nums'] }}>
                {formatValue(r.v)}
              </Text>
            </View>
          ))}
        </View>
      </>
    );
  }

  return (
    <Card className="p-3">
      <View className="flex-row items-center">
        <Text className="text-base font-bold text-primary">{title}</Text>
        {legend.current.rate > 0 ? (
          <Text className="ml-2 text-xs text-muted">{legend.current.rate.toFixed(0)} Hz</Text>
        ) : null}
        <View className="flex-1" />
        {labels.length === 1 ? (
          <Text className="text-base text-primary" style={{ fontVariant: ['tabular-nums'] }}>
            {formatValue(legend.current.values[0] ?? NaN)}
          </Text>
        ) : null}
      </View>

      {/* Two or more series always get a legend: identity never rests on color alone. */}
      {labels.length > 1 ? (
        <View className="mt-2 flex-row flex-wrap">
          {labels.map((label, slot) => {
            const off = hidden.has(slot);
            return (
              <Pressable
                key={label}
                hitSlop={6}
                accessibilityRole="button"
                accessibilityState={{ selected: !off }}
                accessibilityLabel={`${off ? 'Show' : 'Hide'} ${label}`}
                onPress={() =>
                  setHidden((h) => {
                    const next = new Set(h);
                    if (off) next.delete(slot);
                    else next.add(slot);
                    return next;
                  })
                }
                className="mb-1 mr-4 flex-row items-center py-1"
              >
                <View
                  style={{
                    width: 14,
                    height: 3,
                    borderRadius: 2,
                    backgroundColor: chart.series[slot],
                    opacity: off ? 0.25 : 1,
                  }}
                />
                <Text className={`ml-1.5 text-xs ${off ? 'text-muted' : 'text-primary'}`}>{label}</Text>
                <Text className="ml-1 text-xs text-primary" style={{ fontVariant: ['tabular-nums'] }}>
                  {off ? '' : formatValue(legend.current.values[slot] ?? NaN)}
                </Text>
              </Pressable>
            );
          })}
        </View>
      ) : null}

      <GestureDetector gesture={gesture}>
        <View style={{ height: HEIGHT, marginTop: 4 }} onLayout={(e: LayoutChangeEvent) => setWidth(e.nativeEvent.layout.width)}>
          {/* Recessive hairline grid + y labels, from the same scale as the lines. */}
          {ticks.map((t) => (
            <React.Fragment key={t}>
              <View style={{ position: 'absolute', left: LEFT, right: RIGHT, top: y(t), height: 1, backgroundColor: chart.grid }} />
              <Text
                className="text-muted"
                style={{ position: 'absolute', left: 0, width: LEFT - 6, top: y(t) - 7, fontSize: 10, textAlign: 'right', fontVariant: ['tabular-nums'] }}
              >
                {formatTick(t, step)}
              </Text>
            </React.Fragment>
          ))}
          <View style={{ position: 'absolute', left: LEFT, right: RIGHT, top: bottom, height: 1, backgroundColor: chart.axis }} />

          {width > 0 ? (
            <Canvas style={{ position: 'absolute', left: 0, top: 0, width, height: HEIGHT }}>
              {paths.map((p, slot) => (
                <Path
                  key={slot}
                  path={p}
                  color={chart.series[slot]}
                  style="stroke"
                  strokeWidth={2}
                  strokeCap="round"
                  strokeJoin="round"
                />
              ))}
            </Canvas>
          ) : null}

          {!snap ? (
            <Text className="text-sm text-muted" style={{ position: 'absolute', left: LEFT, right: RIGHT, top: TOP + plotH / 2 - 8, textAlign: 'center' }}>
              Waiting for data…
            </Text>
          ) : (
            <>
              <Text className="text-muted" style={{ position: 'absolute', left: LEFT, top: bottom + 3, fontSize: 10 }}>
                −{(windowMs / 1000).toFixed(0)} s
              </Text>
              <Text className="text-muted" style={{ position: 'absolute', right: RIGHT, top: bottom + 3, fontSize: 10, fontVariant: ['tabular-nums'] }}>
                {(tEnd / 1000).toFixed(1)} s
              </Text>
            </>
          )}
          {inspect}
        </View>
      </GestureDetector>

      <Text className="mt-1 text-xs text-muted">
        {inspectX !== null ? 'Paused while inspecting' : 'Long-press the chart to inspect values'}
      </Text>
    </Card>
  );
}
