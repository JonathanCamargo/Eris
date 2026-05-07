# Eris Python driver

Host-side driver for the Eris firmware. Connects to an Arduino/Teensy/SAMD21 running an Eris flavor, configures streaming features, and decodes the COBS-framed serial packets into typed Python records.

## Install

```bash
pip install -e .                 # editable install (recommended during development)
# or
pip install .                    # regular install
# optional extras for SD-card binary parsing & dataframes:
pip install -e ".[analysis]"
```

## Quick start

```python
from eris.eris import Eris

# Open the serial port and select the streaming features.
# 'SineWave' is the canonical built-in feature exposed by every flavor.
e = Eris(features=['SineWave'], format=['float'], port='COM3')
e.start()  # sends S_TIME + S_ON

while True:
    out = e.read()        # returns dict with keys 'D', 'T', 'E'
    for pkt in out['D']:
        print(pkt['SineWave'])

e.stop()                   # sends S_OFF
```

The `features` list must match feature names registered in the flavor's `streaming.cpp` (the `AddFunction` strncmp checks). The `format` list is a parallel list of Construct/struct format strings for decoding each feature.

## Examples

| File | What it does |
|------|--------------|
| `Examples/SineWave.py` | Stream the built-in sine wave; takes `--port` / `--duration` flags |
| `Examples/BandwidthTest.py` | Measure max throughput against an `ErisBandwidthTest` build |
| `Examples/SineWaveROS.py` | Bridge SineWave samples to a ROS1 topic |
| `Examples/BandwidthTest.ipynb` | Interactive bandwidth notebook |

Set `ERIS_PORT` in your environment to skip the `--port` flag in the examples.

## Baud rate

The driver opens the port at 12 Mbps (matches the firmware-side high-speed USB endpoint). The 115200 console baud is only used by the bootloader / `Serial.print` debug output.

## ROS integration

ROS1 (catkin) workspace lives in `ros_ws/`; the message handlers are in `roshandlers/`. See `Examples/SineWaveROS.py` and the top-level repo README for setup.

## Layout

```
drivers/python/
  eris/                 -- package: driver class, custom types
  Examples/             -- runnable scripts and notebooks
  flavors/              -- per-flavor convenience wrappers
  featureextraction/    -- post-processing helpers (optional)
  ros_ws/               -- ROS1 catkin workspace
  roshandlers/          -- ROS1 message bridges
  pyproject.toml        -- packaging
  requirements.txt      -- legacy; use pyproject.toml instead
```
