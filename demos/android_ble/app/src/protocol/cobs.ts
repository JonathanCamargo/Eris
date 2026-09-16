/**
 * Consistent Overhead Byte Stuffing, as used by the firmware's PacketSerial.
 *
 * Every Eris packet on the wire is COBS-encoded and terminated by a single 0x00.
 * The encoding removes all zeros from the payload, so 0x00 can only mean "end of
 * packet" -- that is what lets the app resynchronise after a dropped notification
 * or a mid-stream connect.
 */

/** Decode one frame (delimiter already stripped). Returns null if malformed. */
export function cobsDecode(frame: Uint8Array): Uint8Array | null {
  const out = new Uint8Array(frame.length); // decoded output is never longer
  let read = 0;
  let write = 0;
  while (read < frame.length) {
    const code = frame[read];
    if (code === 0) return null; // zeros cannot appear inside a frame
    read++;
    const end = read + code - 1;
    if (end > frame.length) return null;
    while (read < end) out[write++] = frame[read++];
    // A block shorter than 254 data bytes stood for a zero -- unless it was the
    // last block, whose implied zero is the delimiter itself.
    if (code !== 0xff && read < frame.length) out[write++] = 0;
  }
  return out.slice(0, write);
}

/** Encode `data` (no delimiter appended). Used by the simulator and tests. */
export function cobsEncode(data: Uint8Array): Uint8Array {
  const out = new Uint8Array(data.length + Math.floor(data.length / 254) + 2);
  let codeIndex = 0;
  let write = 1;
  let code = 1;
  for (const b of data) {
    if (b === 0) {
      out[codeIndex] = code;
      codeIndex = write++;
      code = 1;
    } else {
      out[write++] = b;
      code++;
      if (code === 0xff) {
        out[codeIndex] = code;
        codeIndex = write++;
        code = 1;
      }
    }
  }
  out[codeIndex] = code;
  return out.slice(0, write);
}

/**
 * Reassembles COBS frames from an arbitrary byte stream.
 *
 * BLE notifications do not line up with packets: one notification may carry the
 * tail of one packet and the head of the next, or a packet may span several.
 * Feed every notification in order; complete frames come out.
 */
export class FrameSplitter {
  private readonly buffer: Uint8Array;
  private length = 0;
  private overflowed = false;

  /** Frames discarded because they outgrew the buffer (a lost delimiter). */
  overflows = 0;

  constructor(maxFrame = 8192) {
    this.buffer = new Uint8Array(maxFrame);
  }

  feed(bytes: Uint8Array, onFrame: (frame: Uint8Array) => void): void {
    for (const b of bytes) {
      if (b === 0) {
        if (!this.overflowed && this.length > 0) onFrame(this.buffer.slice(0, this.length));
        this.length = 0;
        this.overflowed = false;
      } else if (this.length < this.buffer.length) {
        this.buffer[this.length++] = b;
      } else if (!this.overflowed) {
        this.overflowed = true; // drop until the next delimiter resyncs us
        this.overflows++;
      }
    }
  }

  reset(): void {
    this.length = 0;
    this.overflowed = false;
  }
}
