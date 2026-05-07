"""Read SineWave samples streamed from an Eris-flavored MCU.

Usage:
    python SineWave.py [--port COM3] [--duration 10] [--rate 10]

Defaults to ERIS_PORT environment variable, then platform-specific guess.
"""
import argparse
import os
import sys
from threading import Thread
from time import sleep

import numpy as np

from eris.eris import Eris


def default_port():
    return os.environ.get(
        "ERIS_PORT",
        "COM3" if sys.platform.startswith("win") else "/dev/ttyACM0",
    )


def main():
    parser = argparse.ArgumentParser(description="Stream a SineWave from Eris firmware.")
    parser.add_argument("--port", default=default_port(),
                        help="Serial port (default: $ERIS_PORT or platform guess)")
    parser.add_argument("--duration", type=float, default=10.0,
                        help="Total run time in seconds (default: 10)")
    parser.add_argument("--rate", type=float, default=10.0,
                        help="Read polling rate in Hz (default: 10)")
    args = parser.parse_args()

    e = Eris(['SineWave'], ['float'], [5], args.port)

    def print_data(data):
        d = e.parse(data)
        all_samples = np.array([])
        for packet in d:
            samples = np.array(packet['SineWave'])
            n = np.isnan(samples)
            all_samples = np.concatenate([all_samples, samples[~n]])
        for sample in all_samples:
            print(sample)

    print(f"Streaming SineWave from {args.port} for {args.duration}s")
    iterations = int(args.duration * args.rate)
    for _ in range(iterations):
        out = e.read()
        if len(out) > 0:
            t = Thread(target=print_data, args=(out,))
            t.start()
        sleep(1.0 / args.rate)

    e.stop()


if __name__ == "__main__":
    main()
