# DVL / GPS Scripting Notes

Reference material:

- https://williangalvani.github.io/ardupilot-kb/ArduSub/WIP/DVL-GPS%20integration.html

## Scripts

### `sub-gps-dvl-test1-oneshot.lua`

Simple one-shot source switching test.

- starts on GPS
- switches to DVL when GPS degrades and DVL looks healthy
- switches straight back to GPS when GPS recovers

Use this as a baseline. It does not try to reduce jumps on the way back to GPS.

### `sub-gps-dvl-gated-handover.lua`

Jump-aware handover test.

- starts on GPS
- switches to DVL only when DVL is stable
- switches back to GPS only when GPS and the current EKF estimate are already close

Use this when the main goal is to reduce visible jumps during resurfacing.

### `sub-gps-origin-test2.lua`

Origin bootstrap test.

- assumes DVL / ExternalNav is the main horizontal source
- reads GPS in Lua
- sets origin once from GPS if origin is missing and conditions look good

Use this to explore origin/bootstrap behavior, not as the main handover script.

### `sub-nav-diagnostics.lua`

Basic monitor script.

- reports source set, GPS state, visual odom state, origin, and GPS gap
- includes a few manual test parameters for UI testing
- does not change source or origin

Use this to validate that your diagnostics pipeline is visible before testing behavior.

## Surface / Dive Diagnostic Script

The current diagnostic script is:

- `sub-surface-diagnostics.lua`

Its purpose is to help explain large track jumps when the ROV is near the surface or transitioning between GPS-trusted and DVL-trusted navigation.

### What it watches

The script is read-only. It does not switch EKF sources or set origin.

It reports:

- active EKF source set
- whether EKF origin is set
- GPS fix state
- GPS to EKF horizontal gap
- ExternalNav health and quality
- ExternalNav innovation magnitude
- sudden EKF position jumps between updates
- downward rangefinder distance

### Why this is useful

When tracks jump, the important questions are usually:

- Did the EKF source set change at that moment?
- Was GPS already far from the current EKF estimate?
- Was ExternalNav already disagreeing with the EKF?
- Did the EKF estimate itself move suddenly between updates?

This script is intended to make those questions visible.

### Main outputs

Named values:

- `SDI_SRC` - active source set
- `SDI_ORG` - origin set flag
- `SDI_GFIX` - GPS fix status
- `SDI_GAP` - GPS vs EKF gap in meters
- `SDI_VOH` - visual odom health flag
- `SDI_VOQ` - visual odom quality
- `SDI_XINN` - ExternalNav velocity innovation magnitude
- `SDI_JUMP` - EKF jump distance between updates
- `SDI_RNG` - downward rangefinder distance

Text events:

- source changed
- GPS / EKF gap too high
- ExternalNav innovation too high
- EKF jump too large

### Variable reference

| Variable | Meaning | Typical / expected values |
|---|---|---|
| `SDI_SRC` | Active EKF source set | Usually `0`, `1`, or `2` depending on which source set EKF is using |
| `SDI_ORG` | Whether EKF origin is set | `0` = not set, `1` = set |
| `SDI_GFIX` | GPS fix status | `0`/`1` = no useful fix, `2` = 2D, `3` = 3D, `4+` = DGPS/RTK class fixes. If you are seeing `6`, that means a very strong GPS fix class, typically RTK Fixed in MAVLink terms |
| `SDI_GAP` | Horizontal distance between raw GPS location and current EKF location | Near `0` is ideal. Large values mean GPS and EKF disagree |
| `SDI_VOH` | Visual odom / ExternalNav health flag | `0` = unhealthy or missing, `1` = healthy |
| `SDI_VOQ` | Visual odom / ExternalNav quality | Usually `0` to `100`. Higher is better |
| `SDI_XINN` | ExternalNav velocity innovation magnitude | Lower is better. Large values mean ExternalNav is disagreeing with EKF |
| `SDI_JUMP` | EKF position jump between script updates | Near `0` is normal. Large spikes mean the EKF estimate moved suddenly |
| `SDI_RNG` | Downward rangefinder distance | Distance to bottom/surface reference seen by the downward rangefinder |

### Parameters

- `SDIAG_ENABLE` - enable or disable the script
- `SDIAG_RATEHZ` - update rate
- `SDIAG_TXTPER` - periodic text summary interval
- `SDIAG_GAPWRN` - GPS / EKF gap warning threshold in meters
- `SDIAG_JUMPTH` - EKF jump warning threshold in meters
- `SDIAG_INNTH` - ExternalNav innovation warning threshold

### How to use it

1. Run the script by itself first.
2. Dive and resurface normally.
3. Watch `SDI_SRC`, `SDI_GAP`, `SDI_XINN`, and `SDI_JUMP`.
4. If a large map jump happens, check:
	- whether the source changed
	- whether `SDI_GAP` was already large
	- whether `SDI_XINN` was high
	- whether `SDI_JUMP` spiked at the same time

### Interpretation

- Large `SDI_GAP` means GPS and EKF disagree.
- Large `SDI_XINN` means ExternalNav is not fitting the current EKF state well.
- Large `SDI_JUMP` means the EKF estimate itself moved suddenly.
- A source change at the same time as large `SDI_GAP` is a strong candidate for visible track jumps.
