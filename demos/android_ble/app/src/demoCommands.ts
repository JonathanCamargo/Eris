/**
 * The command pad. Each button sends one line to the firmware, exactly as if it
 * were typed in the Arduino Serial Monitor.
 *
 * To add a button for your own flavor: register the command in the firmware
 * (SerialCom::registerCommands, see ErisBLEDemo/serialcommands.cpp) and add a row
 * here. Command names are at most 8 characters, whole lines at most 64.
 */
export interface CommandButton {
  label: string;
  command: string;
}

export interface CommandGroup {
  title: string;
  buttons: CommandButton[];
}

export const DEMO_COMMANDS: CommandGroup[] = [
  {
    title: 'Device',
    buttons: [
      { label: 'Info', command: 'INFO' },
      { label: 'Ping', command: 'PING' },
      { label: 'Status', command: 'STATUS' },
      { label: 'Reset time', command: 'S_TIME' },
    ],
  },
  {
    title: 'Waveform (WAVES)',
    buttons: [
      { label: '0.5 Hz', command: 'FREQ 0.5' },
      { label: '1 Hz', command: 'FREQ 1' },
      { label: '5 Hz', command: 'FREQ 5' },
      { label: 'Amp 1', command: 'AMP 1' },
      { label: 'Amp 3', command: 'AMP 3' },
    ],
  },
  {
    title: 'RGB LED',
    buttons: [
      { label: 'Red', command: 'RGB R' },
      { label: 'Green', command: 'RGB G' },
      { label: 'Blue', command: 'RGB B' },
      { label: 'White', command: 'RGB W' },
      { label: 'Off', command: 'RGB OFF' },
    ],
  },
];

/** Streams offered on first launch. Names must match the firmware's S_F table. */
export const DEFAULT_STREAMS = ['WAVES', 'ANALOG', 'TEMP', 'SINE'];
export const DEFAULT_ENABLED = new Set(['WAVES']);
