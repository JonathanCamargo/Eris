"""Diagnostic: dump the raw BLE NUS byte stream from an Eris board.

Bypasses the Eris driver entirely. Connects, turns streaming on, and prints the
exact bytes each notification delivers, then analyzes framing (0x00 delimiters,
per-frame length, COBS-decodability). Use this to tell apart "bytes are being
dropped" from "the host is mis-framing/mis-decoding a clean stream".

    python raw_ble_dump.py --name Eris-Bare [--seconds 3]
"""
import argparse
import asyncio
import struct

from bleak import BleakClient, BleakScanner

try:
    import cobs.cobs as cobs
except ImportError:
    cobs = None

NUS_RX = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  # host -> device
NUS_TX = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  # device -> host (notify)

chunks = []   # list of raw notification payloads (bytes)


def on_notify(_sender, data):
    chunks.append(bytes(data))


async def main(name, seconds):
    print(f"Scanning for {name!r} ...")
    dev = await BleakScanner.find_device_by_name(name, timeout=10.0)
    if dev is None:
        # maybe an address was passed
        dev = await BleakScanner.find_device_by_address(name, timeout=5.0)
    if dev is None:
        print("Not found.")
        return

    async with BleakClient(dev) as client:
        print(f"Connected (mtu={getattr(client, 'mtu_size', '?')}). Starting stream ...")
        await client.start_notify(NUS_TX, on_notify)
        await client.write_gatt_char(NUS_RX, b"S_OFF\n", response=False)
        await asyncio.sleep(0.3)
        chunks.clear()
        await client.write_gatt_char(NUS_RX, b"S_F SINE\n", response=False)
        await asyncio.sleep(0.3)
        await client.write_gatt_char(NUS_RX, b"S_ON\n", response=False)
        await asyncio.sleep(seconds)
        await client.write_gatt_char(NUS_RX, b"S_OFF\n", response=False)
        await client.stop_notify(NUS_TX)

    # ---- report ----
    print(f"\n{len(chunks)} notifications received")
    sizes = [len(c) for c in chunks]
    if sizes:
        print(f"notification sizes: min={min(sizes)} max={max(sizes)} "
              f"distinct={sorted(set(sizes))}")
    print("first notifications (raw, exactly as delivered):")
    for i, c in enumerate(chunks[:8]):
        print(f"  notif[{i:2}] len={len(c):3}  {c.hex(' ')}")

    stream = b"".join(chunks)
    print(f"\ntotal bytes={len(stream)}  zero-delimiters={stream.count(0)}")

    frames = stream.split(b"\x00")
    print(f"frames (split on 0x00) = {len(frames)} (last is the running tail)\n")
    print("first frames (full hex):")
    for f in frames[:6]:
        print(f"  len={len(f):3}  {f.hex(' ')}")

    if cobs is None:
        print("\n(install the 'cobs' package for frame decoding)")
        return

    # Decode each COBS frame and, for Eris data ('D') packets, pull out the
    # floatSample_t stream (two little-endian floats per sample: timestamp_ms,
    # value). A clean link decodes nearly every frame; truncated frames (the
    # signature of a saturated radio dropping a packet's tail) fail to decode or
    # decode to a length that doesn't match the declared sample count.
    ok = bad = 0
    samples = 0
    timestamps = []
    for f in frames[:-1]:        # last element is the running tail, not a full frame
        if not f:
            continue
        try:
            d = cobs.decode(f)
        except Exception:
            bad += 1
            continue
        ok += 1
        if len(d) >= 2 and d[0] == ord("D"):
            count = d[1]
            body = d[2:]
            n_full = len(body) // 8
            if n_full != count:
                # decoded but body shorter/longer than the header claims
                print(f"  frame: declared {count} samples, body holds {n_full}")
            for k in range(0, n_full * 8, 8):
                ts, _val = struct.unpack_from("<ff", body, k)
                timestamps.append(ts)
            samples += n_full

    print(f"\nCOBS frames: {ok} ok, {bad} bad")
    print(f"data samples decoded: {samples}")
    if len(timestamps) >= 2:
        span = timestamps[-1] - timestamps[0]      # ms, from firmware t0 clock
        if span > 0:
            print(f"timestamp span: {span:.1f} ms over {len(timestamps)} samples "
                  f"-> ~{1000.0 * (len(timestamps) - 1) / span:.1f} Hz on the wire")
        gaps = [timestamps[i + 1] - timestamps[i] for i in range(len(timestamps) - 1)]
        big = [g for g in gaps if g > 50]
        print(f"sample gaps: min={min(gaps):.1f} ms max={max(gaps):.1f} ms "
              f"({len(big)} gap(s) >50 ms -> dropped packets)")


if __name__ == "__main__":
    p = argparse.ArgumentParser(description="Dump raw Eris BLE NUS bytes and analyze framing.")
    p.add_argument("--name", default="Eris-Bare", help="advertised name or address")
    p.add_argument("--seconds", type=float, default=3.0, help="how long to collect streaming")
    args = p.parse_args()
    asyncio.run(main(args.name, args.seconds))
