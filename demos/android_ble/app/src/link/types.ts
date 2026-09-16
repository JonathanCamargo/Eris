/** A byte pipe to an Eris device. Commands go in as lines; bytes come out. */
export interface ErisLink {
  readonly name: string;
  /** Send one command line (the newline is appended). */
  send(line: string): Promise<void>;
  close(): void;
}

export type LinkState =
  | { kind: 'connecting' }
  | { kind: 'ready'; mtu: number }
  /** `reason` is null for a disconnect the user asked for. */
  | { kind: 'disconnected'; reason: string | null };

export interface LinkListener {
  onState(state: LinkState): void;
  /** Called in arrival order. */
  onBytes(bytes: Uint8Array): void;
}

/** Nordic UART Service -- the de-facto "serial port over BLE" Eris uses. */
export const NUS = {
  service: '6e400001-b5a3-f393-e0a9-e50e24dcca9e',
  /** Phone -> device: commands are written here. */
  rx: '6e400002-b5a3-f393-e0a9-e50e24dcca9e',
  /** Device -> phone: packets arrive as notifications. */
  tx: '6e400003-b5a3-f393-e0a9-e50e24dcca9e',
};
