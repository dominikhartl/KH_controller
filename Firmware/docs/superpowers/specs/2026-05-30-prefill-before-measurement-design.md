# Burst-fill before every measurement

**Date:** 2026-05-30
**Status:** Approved (design)

## Goal

Run the same two-phase bubble-clearing fill that the manual **Fill** button performs
(`'f'` command) automatically before *every* measurement, replacing the existing small
in-measurement prefill. Make the fill pattern configurable so HCl consumption per
measurement can be tuned.

## Background — what "Fill" does today

The Fill button (`src/main.cpp:610`, command `'f'`) runs a two-phase burst pattern to
detach and flush air bubbles out of the titration line/tip:

- **Phase 1 (detach):** 10 × 50 µL bursts, 200 ms apart
- **Phase 2 (flush):** 5 × 1 mL pulses, 200 ms apart
- Total **5.5 mL**, dispensed at Gran-burst RPM/accel (`getGranBurstRPM()` /
  `getGranBurstAccel()`).
- 50 µL / 1 mL amounts are hardcoded; converted to motor units via
  `units = µL · calU / (titV · 1000)`.
- Dispensed acid is subtracted from the HCl tank (`subtractHCl`).

The in-measurement prefill today (`src/main.cpp:1078–1101`) is a *different, smaller*
step: one `prefillUnits` dispense (configured "Prefill Volume µL", default 100, at
titration RPM) followed by 5 tip-clear Gran-burst drops. It does **not** do the
5.5 mL bubble flush.

## Confirmed fluidics fact

Fill/prefill acid is **flushed out before the sample is taken** (sample washes rinse it
away). Therefore fill acid:
- **counts** toward HCl tank depletion (real acid consumed), but
- **must not** feed the `hclPart` dilution compensation (it never dilutes the measured
  sample).

## Design

### 1. Shared routine

Extract one function in `src/main.cpp`:

```c
// Two-phase bubble-clearing fill, shared by the Fill button and the
// pre-measurement fill. Detach bursts (FILL_BURST_UL) then flush pulses
// (FILL_PULSE_UL), both at Gran-burst speed, 200 ms apart. Honors
// abortRequested (breaks early, returning units dispensed so far).
// Relies on titrate()'s auto-enable and leaves EN_PIN2 enabled; the
// caller disables it if standalone. Sets ok=false on pump timeout.
int runBurstFill(bool &ok);
```

Both the `'f'` command handler and `measureKH()` call it, so they cannot drift apart
(shared values — see config).

### 2. Behavior in `measureKH()`

Replace the prefill dispense (`main.cpp:1096–1101`) with `runBurstFill()`. Keep the
calibration/titration-config validation and the 5 tip-clear Gran-burst drops
(`main.cpp:1102–1114`) unchanged — the tip-clear has the distinct job of ejecting the
hanging drop immediately before sampling.

On pump timeout during the fill: publish "Error: titration pump timeout during prefill",
disable `EN_PIN2`, clear measuring state, return — same as today's prefill failure.

### 3. Config — two new fields, remove the old one

| UI label | NVS key | Default | Range | Drives |
|---|---|---|---|---|
| Fill detach bursts (×50 µL) | `fill_brst_n` | 10 | 0–30 | Phase 1 count |
| Fill flush pulses (×1 mL) | `fill_pls_n` | 5 | 0–20 | Phase 2 count |

- Both counts shared by the button and per-measurement fill.
- 0 is allowed for either/both (full flexibility; under-priming is the operator's call —
  the 5 tip-clear drops still provide minimal priming).
- New getters/setters in `config_store.{h,cpp}` with clamp + default, mirroring the
  existing `gran_brpm` pattern.
- Per-step volumes become named constants in `include/config.h`:
  `FILL_BURST_UL = 50.0f`, `FILL_PULSE_UL = 1000.0f`.
- **Remove** `prefill_ul` everywhere: `config_store.{h,cpp}` (`getPrefillVolumeUL` /
  `setPrefillVolumeUL`), `web_server.cpp` (POST handler ~585, cfg JSON ~988, status
  snprintf ~1889/1903), `data/www/index.html` (~308), `data/www/app.js` (~388), and the
  native `test/test_native/stub_config_store.cpp` (~109–110).

### 4. HCl accounting (correctness-sensitive)

- **Tank depletion:** subtract fill units + tip-clear units **right after they run**
  (before "Taking sample"), not deferred to the end. Reason: a measurement aborted in
  the wash/titration phase returns early (`main.cpp:1134`, `1139`) before the end-of-run
  `subtractHCl`; deferring would lose up to 5.5 mL of tank accounting per abort. The
  titration `units` continue to be subtracted at the end of the run as today.
- **`hclPart` dilution compensation (`main.cpp:1718`):** driven by titration `units`
  **only**. Drop the `prefillUnits` term — flushed fill acid never dilutes the sample.
  This also corrects the pre-existing minor over-removal caused by counting the old
  ~100 µL prefill.

### 5. `'f'` command handler

Becomes a thin wrapper:

```c
case 'f': {
  publishMessage("Filling (burst mode)");
  bool ok = true;
  int u = runBurstFill(ok);
  subtractHCl(u);
  if (!ok) publishError("Error: titration pump timeout during fill");
  else     publishMessage("Fill done");
  digitalWrite(EN_PIN2, HIGH);   // standalone: disable after
  broadcastState();
  break;
}
```

### 6. Web UI

Replace the single "Prefill Volume (uL)" input in `index.html` with two integer inputs
("Fill detach bursts (×50 µL)", "Fill flush pulses (×1 mL)"); update `app.js` `setInput`
wiring and the config POST/GET/status string in `web_server.cpp`.

## Testing

- **Native unit suite:** update `stub_config_store.cpp` (drop prefill stubs, add new
  count stubs) and `test_volume_calc.cpp` as needed so `pio test -e native` builds and
  passes.
- **On-device manual checklist** (operator-run; firmware author cannot flash):
  1. Fill button at defaults dispenses ~5.5 mL (10×50 µL + 5×1 mL) — unchanged.
  2. Starting a measurement performs the same fill before "Taking sample".
  3. HCl tank gauge drops by the dispensed amount after a measurement.
  4. Lowering "flush pulses" reduces per-measurement acid use proportionally.
  5. Setting both counts to 0 → measurement runs with only the 5 tip-clear drops.
  6. Abort mid-measurement still depletes the tank by the fill amount already dispensed.

## Out of scope

- Separate config for button vs. per-measurement (explicitly chosen: shared values).
- Configurable per-step volumes (50 µL / 1 mL stay fixed constants).
- Fixing HCl-accounting on the *post-titration* abort paths (pre-existing; unchanged).
