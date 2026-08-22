"""Read SineWave samples streamed from an Eris nRF52 board over Bluetooth LE.

This is the BLE twin of SineWave.py. It talks to firmware built with
-DERIS_USE_BLE (Nordic UART Service) instead of USB serial -- only the
transport changes; the packet protocol is identical.

Uses the 'bleak' BLE library, installed with the eris driver (pip install -e .).

Usage:
    python SineWaveBLE.py [--name Eris-Bare] [--duration 10] [--rate 10]

--name accepts either the advertised BLE name (default 'Eris-Bare', what
BareMinimal advertises when built with -DERIS_BLE_NAME="Eris-Bare") or a BLE
address (MAC on Windows/Linux, UUID on macOS).
"""
import argparse
from time import sleep

from construct import Struct, Float32l

from eris.eris import Eris

# One streamed SineWave sample, matching the firmware's floatSample_t:
#   struct floatSample { float timestamp; float value; };   // 8 bytes, LE
# eris.py wraps this as Array(count, SINE_SAMPLE) behind the uint8 length byte,
# so the decode mirrors StreamSamples<floatSample_t> on the device exactly.
SINE_SAMPLE = Struct("timestamp" / Float32l, "value" / Float32l)


def main():
    parser = argparse.ArgumentParser(description="Stream a SineWave from Eris firmware over BLE.")
    parser.add_argument("--name", default="Eris-Bare",
                        help="BLE advertised name or address (default: Eris-Bare)")
    parser.add_argument("--duration", type=float, default=10.0,
                        help="Total run time in seconds (default: 10)")
    parser.add_argument("--rate", type=float, default=10.0,
                        help="Read polling rate in Hz (default: 10)")
    args = parser.parse_args()

    # 'SINE' must match the firmware's AddFunction() token in streaming.cpp
    # (strncmp("SINE", arg, ...)). This same string is sent as the S_F argument
    # AND used as the decode key below, so it can't be a friendlier alias.
    print(f"Scanning/connecting to BLE device '{args.name}' ...")
    e = Eris(['SINE'], [SINE_SAMPLE], args.name, transport='ble')

    print(f"Streaming SineWave over BLE for {args.duration}s")
    e.start()
    iterations = int(args.duration * args.rate)
    for _ in range(iterations):
        out = e.read()
        for packet in out['D']:
            for s in packet['SINE']:
                print(f"t={s.timestamp:8.1f} ms   value={s.value: .4f}")
        for text in out['T']:
            print('T:', text)
        for err in out['E']:
            print('E:', err)
        sleep(1.0 / args.rate)

    e.stop()


if __name__ == "__main__":
    main()
