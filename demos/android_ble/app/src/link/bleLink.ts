import { PermissionsAndroid, Platform } from 'react-native';
import {
  BleManager,
  ConnectionPriority,
  type Device,
  ScanMode,
  State as BleState,
  type Subscription,
} from 'react-native-ble-plx';
import { asciiBytes, base64ToBytes, bytesToBase64 } from '../protocol/base64';
import { type ErisLink, type LinkListener, NUS } from './types';

let manager: BleManager | null = null;

/** One BleManager for the app's lifetime (created on first use, so tests never touch native code). */
function ble(): BleManager {
  if (!manager) manager = new BleManager();
  return manager;
}

export interface ScanDevice {
  id: string;
  name: string | null;
  rssi: number;
  hasUart: boolean;
}

/** Android 12+ needs Nearby devices; Android 8-11 gates BLE scanning on location. */
export async function requestBlePermissions(): Promise<boolean> {
  if (Platform.OS !== 'android') return true;
  const wanted =
    Number(Platform.Version) >= 31
      ? [PermissionsAndroid.PERMISSIONS.BLUETOOTH_SCAN, PermissionsAndroid.PERMISSIONS.BLUETOOTH_CONNECT]
      : [PermissionsAndroid.PERMISSIONS.ACCESS_FINE_LOCATION];
  const result = await PermissionsAndroid.requestMultiple(wanted);
  return wanted.every((p) => result[p] === PermissionsAndroid.RESULTS.GRANTED);
}

/** Resolves once the adapter reports a definite state (it starts as Unknown). */
export async function bluetoothState(): Promise<BleState> {
  const now = await ble().state();
  if (now !== BleState.Unknown && now !== BleState.Resetting) return now;
  return new Promise((resolve) => {
    const sub = ble().onStateChange((s) => {
      if (s !== BleState.Unknown && s !== BleState.Resetting) {
        sub.remove();
        resolve(s);
      }
    }, true);
  });
}

/** Start scanning. Returns a function that stops it. */
export function scanForDevices(onDevice: (d: ScanDevice) => void, onError: (message: string) => void): () => void {
  // No service filter: some phones drop names from filtered results, and the
  // "all devices" toggle needs everything anyway. The store filters instead.
  ble().startDeviceScan(null, { allowDuplicates: false, scanMode: ScanMode.LowLatency }, (error, device) => {
    if (error) {
      onError(error.message);
      return;
    }
    if (!device) return;
    onDevice({
      id: device.id,
      name: device.localName ?? device.name ?? null,
      rssi: device.rssi ?? -127,
      hasUart: (device.serviceUUIDs ?? []).some((u) => u.toLowerCase() === NUS.service),
    });
  });
  return () => {
    ble().stopDeviceScan();
  };
}

const DESIRED_MTU = 247; // the firmware configures BANDWIDTH_MAX, so a packet fits one notification

/**
 * Eris over Bluetooth LE (Nordic UART Service).
 *
 * connect -> MTU 247 -> discover -> high connection priority -> subscribe to TX -> ready.
 * react-native-ble-plx queues GATT operations itself, so writes can simply be awaited.
 */
export class BleErisLink implements ErisLink {
  private device: Device | null = null;
  private mtu = 23;
  private closed = false;
  private subs: Subscription[] = [];

  constructor(
    private readonly deviceId: string,
    readonly name: string,
    private readonly listener: LinkListener,
  ) {}

  async connect(): Promise<void> {
    this.listener.onState({ kind: 'connecting' });
    try {
      const device = await ble().connectToDevice(this.deviceId, { requestMTU: DESIRED_MTU, timeout: 15000 });
      if (this.closed) {
        await device.cancelConnection().catch(() => undefined);
        return;
      }
      this.device = device;
      await device.discoverAllServicesAndCharacteristics();
      // Shorter connection interval = more notifications per second. Best effort.
      await device.requestConnectionPriority(ConnectionPriority.High).catch(() => undefined);
      this.mtu = device.mtu || DESIRED_MTU;

      const services = await device.services();
      if (!services.some((s) => s.uuid.toLowerCase() === NUS.service)) {
        this.fail('No Nordic UART Service. Is the board running Eris with ERIS_USE_BLE?');
        return;
      }

      this.subs.push(
        ble().onDeviceDisconnected(this.deviceId, (error) => {
          this.fail(error ? `Disconnected: ${error.message}` : 'The board disconnected.');
        }),
        device.monitorCharacteristicForService(NUS.service, NUS.tx, (error, characteristic) => {
          if (error) {
            if (!this.closed) this.fail(`Notifications stopped: ${error.message}`);
            return;
          }
          if (characteristic?.value) this.listener.onBytes(base64ToBytes(characteristic.value));
        }),
      );
      this.listener.onState({ kind: 'ready', mtu: this.mtu });
    } catch (e) {
      // "Operation was cancelled" after a user disconnect is not an error worth showing.
      if (!this.closed) this.fail(`Could not connect: ${(e as Error).message}`);
    }
  }

  async send(line: string): Promise<void> {
    const device = this.device;
    if (!device || this.closed) throw new Error('Not connected');
    const data = asciiBytes(`${line}\n`);
    const chunk = Math.max(20, this.mtu - 3); // ATT header is 3 bytes
    for (let i = 0; i < data.length; i += chunk) {
      const part = bytesToBase64(data.subarray(i, i + chunk));
      await device.writeCharacteristicWithoutResponseForService(NUS.service, NUS.rx, part);
    }
  }

  private fail(reason: string): void {
    if (this.closed) return;
    this.teardown();
    this.listener.onState({ kind: 'disconnected', reason });
  }

  private teardown(): void {
    this.closed = true;
    this.subs.forEach((s) => s.remove());
    this.subs = [];
    ble()
      .cancelDeviceConnection(this.deviceId)
      .catch(() => undefined);
    this.device = null;
  }

  close(): void {
    if (this.closed) return;
    this.teardown();
    this.listener.onState({ kind: 'disconnected', reason: null });
  }
}
