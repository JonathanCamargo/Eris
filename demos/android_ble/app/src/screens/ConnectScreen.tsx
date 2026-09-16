import { Banner, Button, Card, colors, Label } from '@robiolab/native-ui';
import * as React from 'react';
import { ActivityIndicator, ScrollView, Switch, Text, View } from 'react-native';
import { useShallow } from 'zustand/react/shallow';
import type { ScanDevice } from '../link/bleLink';
import {
  connect,
  connectSimulator,
  setShowAllDevices,
  startScan,
  stopScan,
  useEris,
} from '../store/erisStore';

export function ConnectScreen() {
  const { connection, deviceName, lastError, scanning, showAllDevices, devices } = useEris(
    useShallow((s) => ({
      connection: s.connection,
      deviceName: s.deviceName,
      lastError: s.lastError,
      scanning: s.scanning,
      showAllDevices: s.showAllDevices,
      devices: s.devices,
    })),
  );
  const idle = connection === 'idle';

  return (
    <View className="flex-1 bg-background">
      {lastError ? <Banner variant="error" message={lastError} /> : null}

      <ScrollView contentContainerStyle={{ padding: 16, paddingBottom: 32 }}>
        <Text className="text-2xl font-bold text-primary">Connect to an Eris board</Text>
        <Text className="mt-2 text-base text-muted">
          Flash ErisBLEDemo (demos/android_ble/firmware) to a Seeed XIAO nRF52840 and power it. Any Eris
          flavor built with ERIS_USE_BLE works too.
        </Text>

        {connection === 'connecting' ? (
          <Card className="mt-4 flex-row items-center p-4">
            <ActivityIndicator color={colors.primary} />
            <Text className="ml-3 text-base text-primary">Connecting to {deviceName}…</Text>
          </Card>
        ) : null}

        <View className="mt-6 flex-row items-center">
          <Button
            title={scanning ? 'Stop scan' : 'Scan for boards'}
            variant={scanning ? 'secondary' : 'primary'}
            disabled={!idle}
            onPress={scanning ? stopScan : () => void startScan()}
          />
          <View className="flex-1" />
          <Text className="mr-2 text-sm text-muted">All devices</Text>
          <Switch
            value={showAllDevices}
            onValueChange={setShowAllDevices}
            trackColor={{ true: colors.primary, false: colors.surface }}
            thumbColor={colors.background}
          />
        </View>

        {scanning && devices.length === 0 ? (
          <View className="mt-4 flex-row items-center">
            <ActivityIndicator color={colors.primary} />
            <Text className="ml-3 text-sm text-muted">Looking for boards advertising the Nordic UART Service…</Text>
          </View>
        ) : null}

        <View className="mt-4 gap-3">
          {devices.map((d) => (
            <DeviceRow key={d.id} device={d} disabled={!idle} />
          ))}
        </View>

        <View className="mt-8">
          <Label>No board?</Label>
          <Button title="Try the simulator" variant="secondary" disabled={!idle} onPress={connectSimulator} />
          <Text className="mt-2 text-sm text-muted">
            The simulator speaks the same byte protocol as ErisBLEDemo, so everything past the radio works the same.
          </Text>
        </View>
      </ScrollView>
    </View>
  );
}

function DeviceRow({ device, disabled }: { device: ScanDevice; disabled: boolean }) {
  return (
    <Card className="flex-row items-center p-4" onPress={disabled ? undefined : () => connect(device)}>
      <View className="flex-1">
        <Text className="text-base font-bold text-primary">{device.name ?? 'Unnamed device'}</Text>
        <Text className="mt-1 text-xs text-muted">
          {device.id}
          {device.hasUart ? '  ·  UART service' : ''}
        </Text>
      </View>
      <Text className="text-sm text-muted" style={{ fontVariant: ['tabular-nums'] }}>
        {device.rssi} dBm
      </Text>
    </Card>
  );
}
