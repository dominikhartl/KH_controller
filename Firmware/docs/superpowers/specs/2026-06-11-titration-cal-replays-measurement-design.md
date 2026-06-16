# Titration calibration replays the last measurement

**Date:** 2026-06-11
**Status:** Approved (design)

## Goal

Make `calibrateTitrationPump()` dispense the **exact step sequence** of the most recent
successful measurement — same step sizes, same modes, same dwell timing — so the
`units → mL` calibration factor is produced under the identical mode-mix the device
actually uses when measuring. This removes the systematic bias caused by today's
hardcoded, unrepresentative three-phase split.

Until a measurement has been recorded on the new firmware, calibration replays a **real
run captured from the device today and baked into the firmware**, so calibration is
usable immediately after flashing.

## Background — why today's calibration is biased

`calibrateTitrationPump()` (`src/main.cpp:997–1086`) claims to "match measurement's
three-phase titration structure" but uses **hardcoded counts**:

- `MEDIUM_STEPS = 20`, `GRAN_STEPS = 100`, fast = remainder (`main.cpp:1016–1024`).
- Fast phase uses one flat batch size, not the measurement's adaptive
  `computeFastBatch` ramp (`main.cpp:1222–1228`).

A real measurement's stepping is **data-dependent** (fast batches shrink near pH 5.0;
the number of Gran steps depends on buffering) and shifts with config. The medium phase
(`main.cpp:1573–1591`) only runs when `fast_ph > GRAN_REGION_PH`; with the current
device config (`fast_ph = 5.0 = GRAN_REGION_PH`) it never runs.

### Evidence — latest measurement fetched from `khpro.local` (`/api/diagnostics`)

95 steps, 11.37 mL, pH 8.43 → 3.48:

| Phase | Steps | Volume | % vol | Step sizes |
|---|---:|---:|---:|---|
| Fast (pH > 5.0) | 33 | 9.31 mL | 81.9% | ramp 800 → 535/380/218/180/87 → ~80 → 40 µL |
| Medium | **0** | 0 mL | 0% | — |
| Gran (pH ≤ 5.0) | 62 | 2.06 mL | 18.1% | uniform ~33 µL |

Today's calibration would instead push **0.80 mL through medium** (a mode the measurement
never uses) and **2.47 mL through gran** (~20% too much). Because per-mode delivery
differs (`experiments/single_mode_dispense`: fast ≈ 0.773–0.80 vs gran ≈ 0.772–0.781
µL/unit) and even varies with step *size* via each step's accel/decel ramp, this
mode-mix error biases the calibration factor.

## Unit-accounting fact (correctness-sensitive)

`unitsPerUL = cal_units / (titration_volume × 1000)`
(`config_store.cpp:89,93`). The user weighs the dispensed acid and enters
`titration_volume`; `cal_units` is the unit count those mL correspond to. A replay's
total is the **sum of the replayed steps** (~14.7k in the captured example, *not* the
configured 15,000), so calibration **must overwrite `cal_units` with that sum** or the
ratio is wrong.

## Design

### 1. Step-program recorder (in `measureKH`)

A new in-RAM buffer records **every** `titrate()` call across the fast loop
(`main.cpp:1476–1522`) and the precise loop (`main.cpp:1567–1649`) — broader than
`dataPoints`, which only keeps pH < 5.0 points.

```c
// Shared type — declared in a header (e.g. gran_analysis.h) so both the recorder
// and include/default_titration_program.h use the same layout.
struct TitrationStep {
  uint16_t units;    // exact stepVol dosed
  uint8_t  mode;     // 0 = fast, 1 = medium, 2 = gran  (drives rpm/accel at replay)
  uint16_t dwellMs;  // actual non-pumping wall-time of this step
                     // (mix delay + Gran stabilization + reading), via millis()
};
static TitrationStep titrationProgram[MAX_PROGRAM_STEPS];  // MAX_PROGRAM_STEPS = 512
static int           titrationProgramLen;
static bool          titrationProgramTruncated;
```

- Reset (`len = 0`, `truncated = false`) at the start of `measureKH()`.
- Per step: capture `t0 = millis()` immediately after `titrate()` returns; after the
  step's mix/stabilization/read completes, append `{units, mode, (uint16)min(millis()-t0,
  0xFFFF)}`. `dwellMs` is what makes "keep full delays" faithful, including variable Gran
  stabilization, with no pH feedback needed during calibration.
- If `len == MAX_PROGRAM_STEPS`, set `truncated = true` and stop appending (keep dosing).
  Sizing: worst case ≈ maxUnits with min steps → a few hundred; `MAX_PROGRAM_STEPS = 512`.
- **DRAM note (impl):** the buffer is `malloc`'d once at boot from the **heap** (~3 KB),
  not placed in static BSS — the DRAM static segment is already near full and a 512-step
  static array overflowed it by ~4.7 KB. `null` alloc → recording disabled, calibration
  falls back to the embedded run. The serialize/parse scratch in web_server.cpp is
  likewise `malloc`/`free`'d transiently rather than held static.

### 2. Persistence

On **successful** completion only — at the existing post-result point where
`appendGranHistory(...)` runs (`main.cpp:148`) — write the program to LittleFS
`/history/last_titration.prog`. **Not** written on abort/error, so a partial run never
poisons the next calibration.

Binary format: header `{ uint32 magic 'KHTP', uint16 version=1, uint16 count,
uint32 srcUnixTs }` followed by `count` × `TitrationStep`. Overwritten each measurement.
`truncated` programs are still written (replay of a representative prefix is acceptable)
but logged.

### 3. Calibration = replay (`calibrateTitrationPump` rewrite)

Load `/history/last_titration.prog` (validate magic/version/count, file size matches
`header + count×sizeof(step)`).

**Replay path (valid program):**

```text
units = 0
for each step:
    rpm/accel = (mode==2) ? {granBurstRPM, granBurstAccel}
                          : {fastPhaseRPM, default accel}   // fast & medium
    if (!titrate(step.units, rpm, ..., accel)) -> timeout: subtractHCl(units), restore, return
    units += step.units
    yielding delay(step.dwellMs)   // calls measurementYield()/ArduinoOTA, abort-aware
    broadcastProgress(units * 99 / programTotalUnits)
subtractHCl(units)
setCalUnits(units)                 // overwrite so the ratio stays correct
setTitrationCalTimestamp(now)
publishMessage("Replayed last measurement: <units> units (~<X> mL est).
                Weigh the dispensed acid and enter it as Titration Volume.")
```

rpm/accel come from **current** config per mode, so calibration tracks pump-setting
changes (`gran_burst_rpm`, etc.); only the size/mode *distribution* is inherited from the
measurement. The replayed `step.units` are dosed verbatim (they define the dispensed
total).

**Default program (missing/invalid program file — e.g. fresh flash):** replay the
embedded `DEFAULT_TITRATION_PROGRAM` from `include/default_titration_program.h` — a
**real captured run**, not a synthetic approximation. Identical replay code path; just a
different source array. Message: `"No recorded measurement yet — replayed the embedded
reference run; run a measurement first for a run-specific calibration."` Still sets
`cal_units` to the embedded total.

#### 3a. The embedded default program

`include/default_titration_program.h` holds the run fetched from `khpro.local`
(`/api/diagnostics`) on **2026-06-11** (device fw_build `Jun 10 2026 18:10:18`,
`cal_units=15000`, `titration_vol=11.59`):

- **95 steps, 14,731 units (~11.38 mL); 33 fast + 62 gran, 0 medium.**
- Reconstructed per step: `units` from the curve's mL deltas (`round(Δml·calU/titV)`),
  `mode` from the pre-step pH (fast > 5.0, gran ≤ 5.0), `dwellMs` synthesized from the
  fast/gran mix-delay formulas plus the recorded Gran `stabMs`.
- The header carries a provenance comment (source endpoint, fw_build, timestamp). A tiny
  regenerator script is kept under `experiments/` (or documented inline) so the embedded
  run can be refreshed from any future `/api/diagnostics` dump. It is a one-time bootstrap
  — the first recorded measurement overwrites it with `/history/last_titration.prog`.

### 4. Abort / progress / watchdog

Mirror today's calibration safety (`main.cpp:1027–1078`): `broadcastProgress` across the
replay, `measurementYield()` inside the dwell loop, and on abort →
`subtractHCl(unitsSoFar)`, `digitalWrite(EN_PIN2, HIGH)`, `broadcastProgress(100)`,
return.

## Testing

- **Native unit suite** (`pio test -e native`): round-trip
  serialize → bytes → deserialize of a known `TitrationStep[]` (header fields, count,
  per-step fidelity, truncation flag); and a sanity check on the embedded
  `DEFAULT_TITRATION_PROGRAM` (parses, 95 steps, total units in range, contains no
  medium-mode steps). Add stubs as needed so the suite still builds.
- **On-device manual checklist** (operator-run; firmware author cannot flash):
  1. Run a measurement; confirm `/api/diagnostics` shows the curve as before.
  2. Calibrate; confirm it replays the **same step count and total units** as that
     measurement (no medium steps when `fast_ph = 5.0`).
  3. `cal_units` after calibration equals the replayed sum; weighed `titration_volume`
     gives µL/unit in the 0.77–0.78 band (gran-dominated tail) consistent with
     `single_mode_dispense`.
  4. Abort mid-replay → HCl tank depletes by the amount dispensed so far; EN disabled.
  5. Calibrate on a fresh flash (no program file) → replays the embedded reference run
     (95 steps, ~11.4 mL) + the "embedded reference run" message; still completes and sets
     `cal_units`.

## Out of scope

- Persisting the program across firmware *reflashes* of LittleFS (a reflash that wipes
  `/history` falls back to the embedded reference run — acceptable, by design).
- Normalizing the replay to a fixed `cal_units` target (explicitly chosen: replayed sum
  defines `cal_units`).
- Replaying multiple times for extra mass (single ~11 mL replay chosen).
- Changing the measurement titration logic itself — only *recording* is added there.
