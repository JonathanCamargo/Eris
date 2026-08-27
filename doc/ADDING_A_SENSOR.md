# Adding a Sensor to Eris

Everything you need to get a new sensor streaming — and the mistakes that fail
*silently* if you skip them.

This is the recipe for the common case: a sensor you read on a fixed schedule.
If that is not your sensor, read step 0 and then stop.

---

## 0. Is your sensor a poll?

Answer this before writing anything. It decides whether you get the easy path or
write a thread by hand, and trying to force the wrong one is how people lose an
afternoon.

| Your sensor | Use |
|---|---|
| You want a value every N milliseconds, and you get it by *asking* the device — analog reads, most I²C and SPI chips, anything you would put in `loop()` with a delay | **`ERIS_SENSOR`.** Read on. |
| The data decides when: a DRDY/INT line says data is ready; or it arrives by DMA, in bursts, at several rates at once, or gated on something other than time | **Write the thread yourself.** See below. |

If you are in the second row, **do not bend the macro to fit.** Write the thread
with `ERIS_THREAD_WA` / `ERIS_THREAD_FUNC` / `eris_thread_create`, and append to
your own `ErisBuffer` from wherever the data actually arrives.

Every other Eris tool still applies — buffers, the streaming templates, the
packet layer, the error handler. Only the polling loop is missing, because your
data is not polled. `ErisADS1299` is the worked interrupt-driven example.

Everything below assumes the first row. The running example is a three-axis
force sensor on the analog inputs at 100 Hz.

---

## 1. Describe the sample type

Two things go in your flavor's `customtypes.h`: the struct, and a descriptor
saying what its fields are. **Keep them touching** — that adjacency is the whole
safety mechanism.

```c
// Every Eris sample starts with a float timestamp. The macro fills it in.
typedef struct {
  float timestamp;
  float fx, fy, fz;
} ForceSample_t;

ERIS_DESCRIBE_BEGIN(ForceSample_t)
  ERIS_TIME (ForceSample_t, timestamp)
  ERIS_FLOAT(ForceSample_t, fx)
  ERIS_FLOAT(ForceSample_t, fy)
  ERIS_FLOAT(ForceSample_t, fz)
ERIS_DESCRIBE_END(ForceSample_t)
```

### If your sample is a channel array

The `float timestamp; float ch[N];` shape is most of Eris, and it has a
one-liner. Use it instead of listing channels by hand — it reads the same
constant the struct does, so raising the channel count updates the descriptor
automatically:

```c
typedef struct {
  float timestamp;
  float ch[FORCE_NUMCHANNELS];
} ForceSample_t;

ERIS_DESCRIBE_CHANNELS(ForceSample_t, ch, FORCE_NUMCHANNELS)
```

Fields come out named `ch0`, `ch1`, … `chN-1`. The second argument is the member
name, so `float value[N]` works too — pass `value`.

### Why it is worth the four lines

A described type gets Serial Plotter output, Serial Monitor text, and a
self-configuring Python driver for free. An undescribed one gets none of them
and answers `UNDESCRIBED` to the `DESC` command, which forces whoever uses it to
hand-write the layout on the host — the thing this exists to stop.

`ERIS_TIME` is not decorative. Timestamps are marked so they can be *excluded*
from plot output: a monotonically rising trace destroys the Serial Plotter's
autoscale and flattens every real signal to a flat line.

> **Trap — caught at build time.** `ERIS_DESCRIBE_END` checks that the described
> fields account for every byte of the struct. Add a field and forget to
> describe it and the build stops. That is deliberate: a silently missing column
> is very hard to notice in recorded data.

---

## 2. Declare the sensor

One declaration in `force.cpp`. You write the device reads; the macro writes
everything else.

```c
#include "Eris.h"
#include <eris_sensor.h>

ERIS_SENSOR(Force, ForceSample_t, 100) {
  s.fx = analogRead(ERIS_ADC(0)) * FORCE_SCALE;
  s.fy = analogRead(ERIS_ADC(1)) * FORCE_SCALE;
  s.fz = analogRead(ERIS_ADC(2)) * FORCE_SCALE;
  // false = skip this tick, append nothing (device absent, no new data)
  return true;
}
```

That generates `Force::buffer`, `Force::start()`, a working area, the stack
tier, the thread priority, a drift-free 100 Hz loop, and the timestamp. You
choose a name, a type, and a rate.

> **Trap — silent until it isn't.** Use `ERIS_ADC(n)`, never `A1` / `A2`. ESP32
> has no `A1`, `A2`, `A8` or `A9`. Code using the `A*` aliases compiles on
> Teensy and then will not build, or reads the wrong pin, on another board.

### Several devices on one bus

Two chips on the same I²C bus need **one** thread, not two — devices sharing a
bus must have a single owner, or their transactions corrupt each other.
`ERIS_SENSOR_MULTI` polls channels in sequence from one thread and gives each
its own buffer:

```c
ERIS_SENSOR_MULTI(Force, ForceSample_t, 100, 2) {
  if (!ok[ch]) return false;          // absent device: skip
  s.fx = dev[ch]->readX();
  s.fy = dev[ch]->readY();
  s.fz = dev[ch]->readZ();
  return true;
}
```

This generates `Force::buffer[2]`. All channels in a tick share one timestamp,
because they *are* one tick. `ErisMPU` is the reference — two MPU9250s, one
thread.

> **Trap — caught at build time.** Your rate must divide 1000 evenly, so the
> period lands on a whole scheduler tick. 250, 200, 125, 100, 50, 20, 10 are
> fine; 300 and 60 are not. The macro `static_assert`s this, so you find out at
> compile time rather than by wondering why your data is 8% slow.

---

## 3. Register it for streaming

Two edits in `streaming.cpp`, plus a declaration in `streaming.h`.

First the stream function:

```c
void ForceStream(){
  // The "FORCE" label prefixes field names in text mode: FORCE.fx, FORCE.fy
  StreamSamples<ForceSample_t, FORCE_TXBUFFERSIZE>(Force::buffer, packet, "FORCE");
}
```

Then a branch in `AddFunction` so `S_F FORCE` finds it:

```c
else if (!strncmp("FORCE", arg, MAXSTRCMP)){
  streamfnc[Nfunctions] = &ForceStream;
  Nfunctions = Nfunctions + 1;
}
```

And declare `ForceStream()` in `streaming.h`.

> **Trap — the one that will get you.** `MAXSTRCMP` is **4 or 5** depending on
> the flavor, and feature names are compared only that far. `FORCE_X` and
> `FORCE_Y` both match on `FORCE` — whichever branch comes first wins, silently,
> and `S_F FORCE_Y` streams X.
>
> Keep feature names distinct within the first four characters. `FX`/`FY`/`FZ`
> is safe; `FORCE_X`/`FORCE_Y` is not.

> **Trap — a confusing error message.** Declare your stream function in
> `streaming.h`, and do not give it the same name as your sensor namespace. If
> the declaration is missing, `&ForceStream` earlier in the file binds to the
> *namespace* instead, and you get `expected primary-expression before ';'`,
> which points nowhere useful.

---

## 4. Start it

One line in the flavor's `.ino`, inside `start()`:

```c
void start(){
  eriscommon::setPrintPacketMode(true);
  Heartbeat::start();
  Error::start();
  t0 = micros();

  Force::start();                // generated by ERIS_SENSOR

  SerialCom::start();

  // Boot streaming as text, so opening Serial Monitor just shows data
  SerialCom::bootDefaults("FORCE", true);
}
```

If your sensor needs hardware brought up first — `Wire.begin()`, a device
`begin()`, calibration — put that in your own function and call the generated
`start()` at the end of it. `ErisMPU` calls this `IMU::begin()`.

`bootDefaults` is optional but recommended. Without it, someone has to type
`S_F FORCE` then `S_ON` before seeing anything. With it, they plug the board in
and data appears.

---

## 5. See the data

### Serial Monitor / Serial Plotter — no host code

Open the Arduino Serial Monitor at 115200. You get one line per tick:

```
FORCE.fx:0.213,FORCE.fy:-0.044,FORCE.fz:1.902
```

Switch to the Serial Plotter and every signal is graphed and labelled, because
`label:value` is exactly what it parses. Send `S_MODE ASCII TIME` if you want
timestamps in the text.

> **Text mode is a monitor, not a log.** It prints only the **newest** sample
> per feature, at 20 Hz. The rest are drained from the buffer but never
> rendered — at 100 Hz into a 20 Hz display there is nowhere for them to go.
> Never use text mode for data you intend to keep.

### Python — for data you keep

The binary path streams every sample. The driver asks the firmware for its own
layout, so you do not declare one:

```python
from eris.eris import Eris

e = Eris(['FORCE'], port='COM3')   # layout fetched via DESC
e.start()
while True:
    out = e.read()
    for sample in out['D'][0]['FORCE']:
        print(sample.timestamp, sample.fx, sample.fy, sample.fz)
```

This only works if you did step 1. An undescribed type forces you back to
passing an explicit `format=`, hand-written to match your struct — and drifting
from it the first time someone adds a field.

---

## Reference

### What the macro gives you

| You write | You get |
|---|---|
| `ERIS_SENSOR(Name, T, Hz)` | `Name::buffer`, `Name::start()`, thread, stack tier, priority, drift-free loop, timestamp |
| `ERIS_SENSOR_MULTI(Name, T, Hz, N)` | Same, plus `Name::buffer[N]`; body receives `ch`; one thread for all channels |
| `ERIS_SENSOR_EX(…, STACK, PRIO)` | Same, with the stack tier and priority you pick |

### Valid rates

Any positive divisor of 1000:

| Hz | Period | Typical use |
|---:|---:|---|
| 1000 | 1 ms | Ceiling; leaves little room for the bus |
| 250 | 4 ms | IMU (ErisMPU) |
| 100 | 10 ms | Force, analog, most lab sensors |
| 50 | 20 ms | Slow analog, temperature |
| 10 | 100 ms | Housekeeping, battery |

### Descriptor field macros

| Macro | C type | Notes |
|---|---|---|
| `ERIS_TIME` | `float` | The timestamp. Excluded from plots. |
| `ERIS_FLOAT` | `float` | The usual case. |
| `ERIS_U8` / `ERIS_I8` | `uint8_t` / `int8_t` | |
| `ERIS_U16` / `ERIS_I16` | `uint16_t` / `int16_t` | Raw ADC counts. |
| `ERIS_U32` / `ERIS_I32` | `uint32_t` / `int32_t` | |
| `ERIS_META_U8` | `uint8_t` | Status flags — carried, never plotted. |
| `ERIS_PAD` | — | One byte of padding. See below. |
| `ERIS_DESCRIBE_CHANNELS(T, MEMBER, N)` | — | Whole descriptor for `timestamp` + `MEMBER[N]` floats. |

---

## When the build stops

### “does not account for every byte of the struct”

Your struct and its descriptor disagree. Either you added a field and did not
describe it, or the compiler inserted padding.

For padding, reorder the struct largest-type-first — put every `float` and
`int32_t` before any `uint8_t`. If a gap is unavoidable, declare it with one
`ERIS_PAD` per byte.

This error is doing you a favour twice over: the binary protocol assumes a
packed layout, so silent padding would already have been corrupting your
host-side decode. (`boolSample_t` in `ErisTapok2` is a real example — `bool` in
a 4-byte-aligned struct means three padding bytes that the wire format has
always been carrying.)

### “rate must be a positive divisor of 1000 Hz”

Pick a rate from the table above. If you genuinely need 60 Hz, poll at 100 and
decimate in your own code, or write the thread by hand.

### “expected primary-expression before ';'”

Your stream function is not declared in `streaming.h`, so its name is resolving
to a namespace. See step 3.

### Errors pointing inside `Print.h`

You have used a name Arduino already defines as a macro. `BIN`, `DEC`, `HEX` and
`OCT` are all taken, in every core — the preprocessor runs before any namespace
or `enum class` can protect you. Rename your identifier.

### Data looks right but arrives too slowly

You are probably reading it in text mode, which shows 20 samples a second no
matter what rate you set. Send `S_MODE BIN` and use the Python driver.

---

## Where things live

| What | Where |
|---|---|
| Reference flavor — two devices, one thread, described types, text-mode boot | `Firmware/Flavors/ErisMPU` |
| Interrupt-driven example | `Firmware/Flavors/ErisADS1299` |
| `ERIS_SENSOR`, `ERIS_SENSOR_MULTI` | `eriscommon/src/eris_sensor.h` |
| `ERIS_DESCRIBE_*`, `ERIS_DESCRIBE_CHANNELS` | `eriscommon/src/eris_descriptor.h` |
| ASCII mode, `DESC` | `eriscommon/src/eris_ascii.h` |
| Serial command reference | [`doc/COMMANDS.md`](COMMANDS.md) |
| Port/RTOS/build war stories | [`lessons_learned.md`](../lessons_learned.md) |

Both headers carry the full contract in comments — they are the authority if
this document and the code disagree.

Before adding a new *flavor* that differs from an existing one only by which
sensors are attached, ask whether it should be a sensor in an existing flavor
instead.
