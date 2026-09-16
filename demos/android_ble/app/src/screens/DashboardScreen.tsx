import { Button, Card, Chip, colors, EmptyState, Label, SegmentedControl, TextField } from '@robiolab/native-ui';
import * as React from 'react';
import { ActivityIndicator, ScrollView, Text, View } from 'react-native';
import { useShallow } from 'zustand/react/shallow';
import { DEMO_COMMANDS } from '../demoCommands';
import { FeatureCharts } from '../plot/LiveChart';
import {
  addStream,
  clearConsole,
  clearPlots,
  type ConsoleLine,
  sendCommand,
  setStreaming,
  setWindow,
  toggleStream,
  useEris,
} from '../store/erisStore';

const WINDOWS = [
  { value: '2000', label: '2 s' },
  { value: '5000', label: '5 s' },
  { value: '10000', label: '10 s' },
];

export function DashboardScreen() {
  const live = useEris((s) => s.live);
  const windowMs = useEris((s) => s.windowMs);
  const connection = useEris((s) => s.connection);

  return (
    <ScrollView className="flex-1 bg-surface" contentContainerStyle={{ padding: 16, paddingBottom: 32, gap: 16 }}>
      <LinkSection />
      <StreamsSection />

      {live.length === 0 && connection === 'ready' ? (
        <EmptyState title="No streams selected" message="Tick a stream above to start plotting." />
      ) : null}
      {live.map((schema) => (
        <FeatureCharts key={schema.feature} schema={schema} windowMs={windowMs} />
      ))}

      <CommandsSection />
      <ConsoleSection />
    </ScrollView>
  );
}

function LinkSection() {
  const { connection, streaming, live, stats, windowMs } = useEris(
    useShallow((s) => ({
      connection: s.connection,
      streaming: s.streaming,
      live: s.live,
      stats: s.stats,
      windowMs: s.windowMs,
    })),
  );
  const configuring = connection === 'configuring';
  const active = streaming && live.length > 0;

  return (
    <Card className="p-4">
      <Label>Link</Label>
      <View className="flex-row items-center">
        {configuring ? (
          <ActivityIndicator size="small" color={colors.primary} />
        ) : (
          <View
            className={active ? 'bg-primary' : 'border-2 border-muted'}
            style={{ width: 10, height: 10, borderRadius: 5 }}
          />
        )}
        <Text className="ml-2 text-base font-bold text-primary">
          {configuring ? 'Configuring streams…' : active ? 'Streaming' : 'Paused'}
        </Text>
      </View>
      <Text className="mt-1 text-xs text-muted" style={{ fontVariant: ['tabular-nums'] }}>
        {stats.kBytesPerSec.toFixed(1)} kB/s · {stats.packetsPerSec.toFixed(0)} packets/s · {stats.samplesPerSec.toFixed(0)}{' '}
        samples/s
        {stats.badFrames + stats.undecodable > 0 ? ` · ${stats.badFrames} bad, ${stats.undecodable} skipped` : ''}
      </Text>

      <View className="mt-4 flex-row gap-3">
        <Button
          className="flex-1"
          title={streaming ? 'Pause stream' : 'Start stream'}
          variant={streaming ? 'secondary' : 'primary'}
          onPress={() => setStreaming(!streaming)}
        />
        <Button title="Clear plots" variant="ghost" onPress={clearPlots} />
      </View>

      <View className="mt-3">
        <SegmentedControl options={WINDOWS} value={String(windowMs)} onChange={(v) => setWindow(Number(v))} />
      </View>
    </Card>
  );
}

function StreamsSection() {
  const streams = useEris((s) => s.streams);
  const busy = useEris((s) => s.connection === 'configuring');
  const [adding, setAdding] = React.useState(false);
  const [name, setName] = React.useState('');
  const [error, setError] = React.useState<string | undefined>();

  const submit = () => {
    if (addStream(name)) {
      setAdding(false);
      setName('');
      setError(undefined);
    } else {
      setError('Use A–Z, 0–9 or _, and a name not already listed.');
    }
  };

  return (
    <Card className="p-4">
      <Label>Streams</Label>
      <Text className="text-sm text-muted">
        Tick to stream. The app sends S_F, then asks the firmware (DESC) how to decode each stream.
      </Text>
      <View className="mt-3 flex-row flex-wrap gap-2">
        {streams.map((t) => (
          <Chip key={t.name} label={t.name} selected={t.enabled} onPress={busy ? undefined : () => toggleStream(t.name)} />
        ))}
        {!adding ? <Chip label="+ Add" onPress={busy ? undefined : () => setAdding(true)} /> : null}
      </View>

      {adding ? (
        <View className="mt-3">
          <TextField
            label="Stream name"
            placeholder="e.g. IMU_0"
            value={name}
            onChangeText={(v) => setName(v.toUpperCase().replace(/[^A-Z0-9_]/g, ''))}
            autoCapitalize="characters"
            autoFocus
            onSubmitEditing={submit}
            error={error}
            hint="The name your flavor's streaming.cpp accepts in S_F."
          />
          <View className="mt-3 flex-row gap-3">
            <Button title="Add" size="sm" disabled={!name} onPress={submit} />
            <Button
              title="Cancel"
              size="sm"
              variant="ghost"
              onPress={() => {
                setAdding(false);
                setError(undefined);
              }}
            />
          </View>
        </View>
      ) : null}

      {streams
        .filter((t) => t.problem)
        .map((t) => (
          <Text key={t.name} className="mt-2 text-sm text-destructive">
            ⚠ {t.name}: {t.problem}
          </Text>
        ))}
    </Card>
  );
}

function CommandsSection() {
  const [typed, setTyped] = React.useState('');
  const send = () => {
    if (typed.trim()) sendCommand(typed);
    setTyped('');
  };

  return (
    <Card className="p-4">
      {DEMO_COMMANDS.map((group) => (
        <View key={group.title} className="mb-3">
          <Label>{group.title}</Label>
          <View className="flex-row flex-wrap gap-2">
            {group.buttons.map((b) => (
              <Button key={b.command} title={b.label} size="sm" variant="secondary" onPress={() => sendCommand(b.command)} />
            ))}
          </View>
        </View>
      ))}
      <View className="flex-row items-end gap-3">
        <TextField
          containerClassName="flex-1"
          label="Any command"
          placeholder="e.g. FREQ 2.5"
          value={typed}
          onChangeText={(v) => setTyped(v.slice(0, 64))} // SerialCommand's line buffer
          autoCapitalize="characters"
          returnKeyType="send"
          onSubmitEditing={send}
        />
        <Button title="Send" disabled={!typed.trim()} onPress={send} />
      </View>
    </Card>
  );
}

const PREFIX: Record<ConsoleLine['kind'], string> = { sent: '›', text: ' ', error: 'ERR', app: '·' };
const TONE: Record<ConsoleLine['kind'], string> = {
  sent: 'text-primary font-bold',
  text: 'text-primary',
  error: 'text-destructive',
  app: 'text-muted',
};

function ConsoleSection() {
  const lines = useEris((s) => s.console);
  return (
    <Card className="p-4">
      <View className="flex-row items-center">
        <View className="flex-1">
          <Label>Console</Label>
        </View>
        <Button title="Clear" size="sm" variant="ghost" onPress={clearConsole} />
      </View>
      {lines.length === 0 ? <Text className="text-sm text-muted">Replies from the firmware appear here.</Text> : null}
      {lines.slice(-40).map((line) => (
        <View key={line.id} className="flex-row py-0.5">
          <Text className={`w-8 text-xs ${TONE[line.kind]}`} style={{ fontFamily: 'monospace' }}>
            {PREFIX[line.kind]}
          </Text>
          <Text className={`flex-1 text-xs ${TONE[line.kind]}`} style={{ fontFamily: 'monospace' }}>
            {line.text}
          </Text>
        </View>
      ))}
    </Card>
  );
}
