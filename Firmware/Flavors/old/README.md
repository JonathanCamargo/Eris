# Retired / archived flavors

This folder holds flavors that are **no longer part of the active set**: superseded
by a newer flavor, or whose target hardware is dormant. They are kept here (not
deleted) so their git history and design stay available for reference.

Everything in `old/` is intentionally outside the maintained surface:

- **Not built by CI or PlatformIO.** `platformio.ini` resolves `src_dir` from the
  env name (`Firmware/Flavors/${PIOENV}`), and no env points here.
- **Not swept for drift.** `tools/check_drift.py` and the "keep the shared
  boilerplate in sync" convention do not apply — these are frozen as-is.
- **Not guaranteed to compile** against the current `eriscommon`. A later library
  refactor may have moved on without them; expect to fix includes/symbols if you
  ever revive one.

## How to archive a flavor here

```
git mv Firmware/Flavors/<Name> Firmware/Flavors/old/<Name>
```

Then remove its `[env:<Name>]` from `platformio.ini` (if present) and drop any CI
matrix entry. To revive one, `git mv` it back out and re-add its env.

## Contents

Archived 2026-07-01 (superseded / dormant hardware):

| Flavor | Reason | Successor |
|--------|--------|-----------|
| `ErisADS131` | Older 3-ch ADS131 EMG, v2.0 firmware | `ErisADS1299` (8-ch, v3.0) |
| `ErisTapok`  | Wireless CAN EMG, "untouched, awaiting update" | `ErisTapok2` (adds SD) |
| `ErisBiom`   | Biometrics with older `F_R`/`F_MASK`/`F_WIN` API | `ErisBiom2` (`REG`/`CLASS` API) |
| `ErisLeg`    | Lower-limb exo controller, dormant (awaiting new device assembly) | — |
