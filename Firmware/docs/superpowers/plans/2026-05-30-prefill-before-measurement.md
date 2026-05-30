# Burst-fill before every measurement — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Run the same two-phase bubble-clearing fill the manual "Fill" button performs automatically before every measurement, replacing the small in-measurement prefill, with the fill pattern configurable via two count fields.

**Architecture:** Extract the Fill-button's two-phase pattern (N×50 µL detach bursts + M×1 mL flush pulses, at Gran-burst speed) into one shared `runBurstFill()` routine called by both the `'f'` command handler and `measureKH()`. Two new config counts (`fill_brst_n`, `fill_pls_n`) drive both. Fill acid is flushed before the sample, so it depletes the HCl tank but is removed from the `hclPart` dilution compensation. The old `prefill_ul` config is removed everywhere.

**Tech Stack:** C++ (ESP32 / Arduino / PlatformIO), ESP32 Preferences (NVS), ArduinoJson, vanilla JS + HTML web UI, Unity native unit tests.

> **Commit policy (user instruction):** Do NOT commit per task. Implement all tasks, verify, then make a single commit in the final task.

---

## File Structure

- `include/config.h` — add fill volume/count constants (`FILL_BURST_UL`, `FILL_PULSE_UL`, `FILL_BURST_COUNT`, `FILL_PULSE_COUNT`).
- `src/config_store.h` — replace `getPrefillVolumeUL`/`setPrefillVolumeUL` declarations with `getFillBurstCount`/`setFillBurstCount`/`getFillPulseCount`/`setFillPulseCount`.
- `src/config_store.cpp` — replace the `prefill_ul` getter/setter with the two count getters/setters (NVS keys `fill_brst_n`, `fill_pls_n`).
- `test/test_native/stub_config_store.cpp` — replace the prefill stub defs with the two count stub defs (required so the test build compiles against the new header).
- `test/test_native/test_volume_calc.cpp` — add a regression test for the fill total derived from the new constants.
- `src/main.cpp` — add `runBurstFill()`; rewire the `'f'` command handler to it; replace the prefill block in `measureKH()` and fix HCl accounting.
- `src/web_server.cpp` — replace `prefill_ul` in the config POST handler, the config-GET JSON, and the status snprintf with the two new keys.
- `data/www/index.html` — replace the "Prefill Volume (uL)" input with two count inputs.
- `data/www/app.js` — replace the `prefill_ul` `setInput` wiring with the two count fields.

---

### Task 1: Config constants + native regression test

**Files:**
- Modify: `include/config.h:63` (after `PREFILL_RPM`)
- Test: `test/test_native/test_volume_calc.cpp` (add one test + register it in `run_volume_tests()`)

> **Note on test style:** This file mirrors production formulas with literal values and source-line comments — it does NOT include production headers (`config.h` pulls in `<credentials.h>`, a gitignored stub). Keep the new test literal-based to match the house pattern. The real build-level guardrails are Task 2 (stub/header sync) and the firmware compiles in Tasks 3–5.

- [ ] **Step 1: Add the constants to `include/config.h`**

Insert after line 63 (`static const float PREFILL_RPM = 63.0f; ...`):

```c
static const float FILL_BURST_UL  = 50.0f;    // Phase-1 bubble-detach burst volume (µL)
static const float FILL_PULSE_UL  = 1000.0f;  // Phase-2 bubble-flush pulse volume (µL)
static const int   FILL_BURST_COUNT = 10;     // Default phase-1 detach burst count
static const int   FILL_PULSE_COUNT = 5;      // Default phase-2 flush pulse count
```

- [ ] **Step 2: Add the regression test**

Add this function to `test/test_native/test_volume_calc.cpp`, after the existing `test_removal_ratio_consistent` (or any existing volume test, before the `run_volume_tests()` function):

```c
void test_fill_default_total_volume(void) {
  // Default fill mirrors runBurstFill() in main.cpp and config.h:
  //   FILL_BURST_UL=50, FILL_PULSE_UL=1000, FILL_BURST_COUNT=10, FILL_PULSE_COUNT=5
  float calU = 6000.0f;
  float titV = 9.8f;  // mL per calUnits
  int burstUnits = (int)round(50.0f * calU / (titV * 1000.0f));
  if (burstUnits < 2) burstUnits = 2;
  int pulseUnits = (int)round(1000.0f * calU / (titV * 1000.0f));
  if (pulseUnits < 2) pulseUnits = 2;
  int totalUnits = 10 * burstUnits + 5 * pulseUnits;
  // Convert units back to µL and check it equals the expected 5.5 mL fill.
  float totalUL = (float)totalUnits * titV * 1000.0f / calU;
  TEST_ASSERT_FLOAT_WITHIN(60.0f, 5500.0f, totalUL);  // ~5.5 mL, tolerance for unit rounding
}
```

Register it inside the `run_volume_tests(void)` function in the SAME file (where the existing `RUN_TEST(test_volume_round_trip);` … `RUN_TEST(test_removal_ratio_consistent);` calls are, ~lines 96–103). Add:

```c
  RUN_TEST(test_fill_default_total_volume);
```

- [ ] **Step 3: Run the test to verify it passes**

Run: `pio test -e native`
Expected: PASS — `test_fill_default_total_volume` and all existing tests pass (45 cases total).

---

### Task 2: ConfigStore — replace `prefill_ul` with the two count fields

**Files:**
- Modify: `src/config_store.h:57-58`
- Modify: `src/config_store.cpp:130-131`
- Modify: `test/test_native/stub_config_store.cpp:109-110`

- [ ] **Step 1: Update the header declarations**

In `src/config_store.h`, replace lines 57-58:

```c
  float getPrefillVolumeUL();    // Prefill volume in µL (default 100.0)
  void setPrefillVolumeUL(float ul);
```

with:

```c
  int getFillBurstCount();       // Fill phase-1 detach burst count (×FILL_BURST_UL, default FILL_BURST_COUNT)
  void setFillBurstCount(int n);
  int getFillPulseCount();       // Fill phase-2 flush pulse count (×FILL_PULSE_UL, default FILL_PULSE_COUNT)
  void setFillPulseCount(int n);
```

- [ ] **Step 2: Update the implementation**

In `src/config_store.cpp`, replace lines 130-131:

```c
float ConfigStore::getPrefillVolumeUL()          { return clampf(prefs.getFloat("prefill_ul", 100.0), 10.0f, 500.0f); }
void ConfigStore::setPrefillVolumeUL(float ul)   { prefs.putFloat("prefill_ul", clampf(ul, 10.0f, 500.0f)); }
```

with (uses the existing `clampu32` helper and the new constants from `config.h`, already included by this file):

```c
int ConfigStore::getFillBurstCount()             { return (int)clampu32(prefs.getUInt("fill_brst_n", FILL_BURST_COUNT), 0, 30); }
void ConfigStore::setFillBurstCount(int n)       { prefs.putUInt("fill_brst_n", clampu32((uint32_t)n, 0, 30)); }
int ConfigStore::getFillPulseCount()             { return (int)clampu32(prefs.getUInt("fill_pls_n", FILL_PULSE_COUNT), 0, 20); }
void ConfigStore::setFillPulseCount(int n)       { prefs.putUInt("fill_pls_n", clampu32((uint32_t)n, 0, 20)); }
```

- [ ] **Step 3: Update the native stub (required — defining a removed member would break the test build)**

In `test/test_native/stub_config_store.cpp`, replace lines 109-110:

```c
float ConfigStore::getPrefillVolumeUL() { return 100.0f; }
void ConfigStore::setPrefillVolumeUL(float) {}
```

with:

```c
int ConfigStore::getFillBurstCount() { return 10; }
void ConfigStore::setFillBurstCount(int) {}
int ConfigStore::getFillPulseCount() { return 5; }
void ConfigStore::setFillPulseCount(int) {}
```

- [ ] **Step 4: Verify the native build/link still passes**

Run: `pio test -e native`
Expected: PASS — confirms the stub matches the updated header (the prefill member is fully gone, the new members compile).

---

### Task 3: Add `runBurstFill()` and rewire the `'f'` (Fill) command

**Files:**
- Modify: `src/main.cpp:125` (forward declaration, near `void subtractHCl(int unitsUsed);`)
- Modify: `src/main.cpp` (add function definition next to `subtractHCl`, ~line 860)
- Modify: `src/main.cpp:610-648` (the `case 'f':` handler)

- [ ] **Step 1: Add the forward declaration**

In `src/main.cpp`, just after the existing `void subtractHCl(int unitsUsed);` (line 125), add:

```c
int runBurstFill(bool &ok);
```

- [ ] **Step 2: Add the function definition**

In `src/main.cpp`, immediately after the `subtractHCl(...)` function definition (which ends at ~line 860, just before the `// --- ` section that follows), add:

```c
// Two-phase bubble-clearing fill, shared by the Fill button and the
// pre-measurement fill. Phase 1: FILL_BURST_COUNT-style detach bursts of
// FILL_BURST_UL; Phase 2: flush pulses of FILL_PULSE_UL — both at Gran-burst
// speed, 200 ms apart. Counts come from config. Honors abortRequested
// (breaks early, returning units dispensed so far). Relies on titrate()'s
// auto-enable and leaves EN_PIN2 enabled; the caller disables it if standalone.
// Sets ok=false on pump timeout. Returns total units dispensed (for HCl accounting).
int runBurstFill(bool &ok) {
  ok = true;
  float calU = (float)configStore.getCalUnits();
  float titV = configStore.getTitrationVolume();
  if (calU <= 0 || titV <= 0) { ok = false; return 0; }

  float burstRPM      = configStore.getGranBurstRPM();
  uint32_t burstAccel = configStore.getGranBurstAccel();
  int burstCount = configStore.getFillBurstCount();
  int pulseCount = configStore.getFillPulseCount();
  int burstUnits = max(2, (int)round(FILL_BURST_UL * calU / (titV * 1000.0f)));
  int pulseUnits = max(2, (int)round(FILL_PULSE_UL * calU / (titV * 1000.0f)));
  int dispensed = 0;

  // Phase 1: detach bubbles with small bursts
  for (int i = 0; i < burstCount; i++) {
    if (abortRequested) return dispensed;
    if (!titrate(burstUnits, burstRPM, false, burstAccel)) { ok = false; return dispensed; }
    dispensed += burstUnits;
    if (i < burstCount - 1) delay(200);
  }
  // Phase 2: flush bubbles out with larger pulses
  for (int i = 0; i < pulseCount; i++) {
    if (abortRequested) return dispensed;
    if (!titrate(pulseUnits, burstRPM, false, burstAccel)) { ok = false; return dispensed; }
    dispensed += pulseUnits;
    if (i < pulseCount - 1) delay(200);
  }
  return dispensed;
}
```

- [ ] **Step 3: Rewire the `'f'` command handler**

In `src/main.cpp`, replace the entire `case 'f': { ... }` block (lines 610-648) with:

```c
    case 'f': {
      publishMessage("Filling (burst mode)");
      bool fillOk = true;
      int unitsDispensed = runBurstFill(fillOk);  // titrate() auto-enables EN_PIN2
      if (!fillOk) {
        publishError("Error: titration pump timeout during fill");
      } else {
        publishMessage("Fill done");
      }
      subtractHCl(unitsDispensed);
      digitalWrite(EN_PIN2, HIGH);  // standalone: disable driver after
      broadcastState();
      break;
    }
```

- [ ] **Step 4: Verify the firmware compiles**

Run: `pio run -e serial`
Expected: `SUCCESS` (compiles and links the full firmware; no flashing).

---

### Task 4: Replace the prefill in `measureKH()` and fix HCl accounting

**Files:**
- Modify: `src/main.cpp:1078-1101` (prefill block)
- Modify: `src/main.cpp:1114` (add early HCl subtraction after tip-clear bursts)
- Modify: `src/main.cpp:1688-1692` (end-of-run HCl subtraction)
- Modify: `src/main.cpp:1718` (`hclPart` dilution compensation)

- [ ] **Step 1: Replace the prefill block**

In `src/main.cpp`, replace lines 1078-1101 (from the `// Compute prefill volume in units from µL config` comment through the closing `}` of the `if (!titrate(prefillUnits, ...))` block) with:

```c
  // calU / titV are reused below (tip-clear burst sizing, dilution compensation)
  float calU = (float)configStore.getCalUnits();
  float titV = configStore.getTitrationVolume();
  if (titV <= 0 || calU <= 0) {
    publishError("Error: invalid calibration or titration volume config");
    isMeasuringKH = false; setMeasPhase(0);
    return result;
  }

  // Validate calibration before starting
  if (!isCalibrationValid()) {
    publishError("Error: pH calibration invalid. Re-calibrate with pH 4/7/10 buffers.");
    isMeasuringKH = false; setMeasPhase(0);
    return result;
  }

  // Pre-measurement bubble-clearing fill — same routine as the Fill button.
  // Fill acid is flushed out before the sample, so it counts toward HCl tank
  // depletion but NOT toward the post-titration dilution compensation (hclPart).
  bool fillOk = true;
  int fillUnits = runBurstFill(fillOk);
  if (!fillOk) {
    publishError("Error: titration pump timeout during prefill");
    if (fillUnits > 0) subtractHCl(fillUnits);  // account for acid dispensed before timeout
    digitalWrite(EN_PIN2, HIGH);
    isMeasuringKH = false; setMeasPhase(0);
    return result;
  }
```

(The existing tip-clear burst block at the next lines — `int burstUnits = 0; ... for (int i = 0; i < 5; i++) { ... burstUnits += burstStep; }` — and the `// Keep titration motor enabled after prefill` comment stay unchanged. The local `calU`/`titV` it references are now declared above.)

- [ ] **Step 2: Add early HCl subtraction after the tip-clear bursts**

In `src/main.cpp`, immediately after the tip-clear burst `for` loop closes (the line `}` ending the burst loop at ~line 1114, just before `// Keep titration motor enabled after prefill to prevent suckback`), add:

```c
  // Account for fill + tip-clear acid now: it is flushed before the sample, and
  // doing it here (rather than at end-of-run) ensures it is tracked even when a
  // later abort returns early before the final subtractHCl.
  if (fillUnits + burstUnits > 0) subtractHCl(fillUnits + burstUnits);
```

- [ ] **Step 3: Change the end-of-run HCl subtraction to titration units only**

In `src/main.cpp`, replace lines 1688-1692:

```c
  // Always subtract HCl used (even on error/abort — acid was dispensed regardless).
  // Includes prefill and the tip-clear burst drops, not just the titration `units`.
  if (units + prefillUnits + burstUnits > 0) {
    subtractHCl(units + prefillUnits + burstUnits);
  }
```

with:

```c
  // Subtract titration acid. Fill + tip-clear acid was already accounted for
  // right after sampling (it is flushed before the sample).
  if (units > 0) {
    subtractHCl(units);
  }
```

- [ ] **Step 4: Fix the `hclPart` dilution compensation**

In `src/main.cpp`, replace line 1718:

```c
      hclPart = ((float)(units + prefillUnits) / calU) * titV / samV;
```

with (only titration acid genuinely dilutes the measured sample; flushed fill acid does not):

```c
      hclPart = ((float)units / calU) * titV / samV;
```

- [ ] **Step 5: Verify the firmware compiles**

Run: `pio run -e serial`
Expected: `SUCCESS`. In particular there must be NO `prefillUnits was not declared` errors — confirm the symbol `prefillUnits` no longer appears in `src/main.cpp`:

Run: `grep -n "prefillUnits\|getPrefillVolumeUL" src/main.cpp`
Expected: no output.

---

### Task 5: web_server.cpp — replace `prefill_ul` plumbing with the two count fields

**Files:**
- Modify: `src/web_server.cpp:585` (config POST handler)
- Modify: `src/web_server.cpp:988` (config-GET JSON via ArduinoJson)
- Modify: `src/web_server.cpp:1889` and `src/web_server.cpp:1903` (status snprintf format + args)

- [ ] **Step 1: Update the config POST handler**

In `src/web_server.cpp`, replace line 585:

```c
      else if (strcmp(key, "prefill_ul") == 0) { configStore.setPrefillVolumeUL(value); }
```

with:

```c
      else if (strcmp(key, "fill_brst_n") == 0) { configStore.setFillBurstCount((int)value); }
      else if (strcmp(key, "fill_pls_n") == 0)  { configStore.setFillPulseCount((int)value); }
```

- [ ] **Step 2: Update the config-GET JSON**

In `src/web_server.cpp`, replace line 988:

```c
  cfg["prefill_ul"] = configStore.getPrefillVolumeUL();
```

with:

```c
  cfg["fill_brst_n"] = configStore.getFillBurstCount();
  cfg["fill_pls_n"] = configStore.getFillPulseCount();
```

- [ ] **Step 3: Update the status snprintf format string**

In `src/web_server.cpp` line 1889, replace the format-string fragment:

```c
              "\"drop_ul\":%.1f,\"titration_rpm\":%.1f,\"gran_burst_rpm\":%.1f,\"gran_burst_accel\":%u,\"fast_phase_rpm\":%.1f,\"prefill_ul\":%.1f,"
```

with:

```c
              "\"drop_ul\":%.1f,\"titration_rpm\":%.1f,\"gran_burst_rpm\":%.1f,\"gran_burst_accel\":%u,\"fast_phase_rpm\":%.1f,\"fill_brst_n\":%d,\"fill_pls_n\":%d,"
```

- [ ] **Step 4: Update the matching snprintf argument**

In `src/web_server.cpp` line 1903, replace:

```c
              configStore.getDropVolumeUL(), configStore.getTitrationRPM(), configStore.getGranBurstRPM(), configStore.getGranBurstAccel(), configStore.getFastPhaseRPM(), configStore.getPrefillVolumeUL(),
```

with (one `%.1f` argument becomes two `%d` arguments — order matches the format string):

```c
              configStore.getDropVolumeUL(), configStore.getTitrationRPM(), configStore.getGranBurstRPM(), configStore.getGranBurstAccel(), configStore.getFastPhaseRPM(), configStore.getFillBurstCount(), configStore.getFillPulseCount(),
```

- [ ] **Step 5: Verify the firmware compiles**

Run: `pio run -e serial`
Expected: `SUCCESS`. Confirm no stale references remain:

Run: `grep -rn "prefill_ul\|getPrefillVolumeUL\|setPrefillVolumeUL" src/`
Expected: no output.

---

### Task 6: Web UI — replace the prefill input with the two count inputs

**Files:**
- Modify: `data/www/index.html:308`
- Modify: `data/www/app.js:388`

- [ ] **Step 1: Replace the input in `index.html`**

In `data/www/index.html`, replace line 308:

```html
          <label>Prefill Volume (uL)<input type="number" id="cfg-prefill_ul" step="10" min="10" max="500"></label>
```

with:

```html
          <label>Fill detach bursts (×50 µL)<input type="number" id="cfg-fill_brst_n" step="1" min="0" max="30"></label>
          <label>Fill flush pulses (×1 mL)<input type="number" id="cfg-fill_pls_n" step="1" min="0" max="20"></label>
```

(These stay inside `.config-grid`, so `initConfigInputs()` auto-attaches the `change` listener that sends `{type:'config', key:'fill_brst_n'|'fill_pls_n', value}` — no extra save code needed.)

- [ ] **Step 2: Update the load-wiring in `app.js`**

In `data/www/app.js`, replace line 388:

```js
      setInput('cfg-prefill_ul', d.config.prefill_ul);
```

with:

```js
      setInput('cfg-fill_brst_n', d.config.fill_brst_n);
      setInput('cfg-fill_pls_n', d.config.fill_pls_n);
```

- [ ] **Step 3: Verify no stale web references remain**

Run: `grep -rn "prefill_ul\|prefill" data/www/`
Expected: no output (other than possibly unrelated CSS `fill` matches — confirm none reference `prefill`).

---

### Task 7: Full verification + single commit

**Files:** none (verification + commit only)

- [ ] **Step 1: Run the native test suite**

Run: `pio test -e native`
Expected: all tests PASS, including `test_fill_default_total_volume`.

- [ ] **Step 2: Build the full firmware**

Run: `pio run -e serial`
Expected: `SUCCESS`.

- [ ] **Step 3: Final stale-reference sweep**

Run: `grep -rn "prefill_ul\|PrefillVolume\|prefillUnits" src/ include/ data/ test/`
Expected: no output.

- [ ] **Step 4: Commit (single commit, per user instruction)**

```bash
git add include/config.h src/config_store.h src/config_store.cpp \
        test/test_native/stub_config_store.cpp test/test_native/test_volume_calc.cpp \
        src/main.cpp src/web_server.cpp data/www/index.html data/www/app.js \
        docs/superpowers/specs/2026-05-30-prefill-before-measurement-design.md \
        docs/superpowers/plans/2026-05-30-prefill-before-measurement.md
git commit -m "feat(meas): run Fill-button burst purge before every measurement

Replace the small in-measurement prefill with the same two-phase bubble-
clearing fill the Fill button runs (N×50 µL detach bursts + M×1 mL flush
pulses at Gran-burst speed), via a shared runBurstFill() used by both the
'f' command and measureKH(). Counts are configurable (fill_brst_n /
fill_pls_n, defaults 10/5 = 5.5 mL); the old prefill_ul config is removed.

Fill acid is flushed before the sample, so it depletes the HCl tank but no
longer feeds the hclPart dilution compensation; fill+tip-clear acid is now
subtracted right after sampling so an aborted run still accounts for it.

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

- [ ] **Step 5: On-device manual checklist (operator-run — firmware author cannot flash)**

After flashing firmware (`pio run -e serial -t upload` or OTA) and uploading the filesystem (history-preserving `uploadfs` procedure), verify on the device:

1. Fill button at defaults dispenses ~5.5 mL (10×50 µL + 5×1 mL) — unchanged behavior.
2. Starting a measurement performs the same burst fill before "Taking sample".
3. The HCl tank gauge drops by the dispensed fill amount after a measurement.
4. Lowering "Fill flush pulses" in settings reduces per-measurement acid use proportionally; the value persists across reboot (NVS).
5. Setting both counts to 0 → a measurement runs with only the 5 tip-clear drops (no big fill).
6. Aborting a measurement after the fill still depletes the tank by the fill amount already dispensed.

---

## Self-Review

- **Spec coverage:** Shared `runBurstFill()` (Task 3) ✓; replace in-measurement prefill (Task 4) ✓; two configurable counts with 0 allowed (Tasks 1–2, 5–6) ✓; remove `prefill_ul` everywhere (Tasks 2, 5, 6) ✓; HCl tank counts fill, `hclPart` excludes it, early subtraction for abort-safety (Task 4) ✓; web UI swap (Task 6) ✓; native test + on-device checklist (Tasks 1, 7) ✓.
- **Type consistency:** `runBurstFill(bool &ok) -> int` declared (Task 3 Step 1) and defined (Step 2) identically; callers in Task 3 (`'f'`) and Task 4 (`measureKH`) match. Getter/setter names `getFillBurstCount`/`setFillBurstCount`/`getFillPulseCount`/`setFillPulseCount` identical across header (Task 2.1), impl (2.2), stub (2.3), web_server (Task 5), and `runBurstFill` (Task 3.2). NVS keys `fill_brst_n`/`fill_pls_n` (≤15 chars) consistent between impl and POST handler.
- **Placeholder scan:** none — every code step shows complete content.
