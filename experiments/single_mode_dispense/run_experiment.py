#!/usr/bin/env python3
"""Single-mode dispense test: is the blended pump calibration faithful?

The titration calibration dispenses 59% fast / 8% medium / 33% Gran in one
~13-minute quasi-continuous run. A real titration is ~92% fast / 8% Gran with
~7s pauses between Gran bursts and 25+ min of acid dwell time in the line.
This experiment weighs each dosing mode separately:

  fast      — fast-phase replica (big batches, fast RPM, 200ms apart)
  gran200   — Gran bursts, 200ms spacing  (= what the calibration does)
  gran7s    — Gran bursts, 7s spacing     (= what a real titration does)

Interpretation:
  fast != gran200      -> speed-dependent slip; calibration blend is biased
  gran200 != gran7s    -> dwell-time effect (in-line bubbles/thermal) — the
                          prime suspect for the post-March consistency loss
  repeated gran7s SD   -> direct measurement of run-to-run delivery variance
                          (the slope-ratio scatter inferred ±2-3%)

Each dispense is 7500 units (~4.9 mL at cal 15000u/9.84mL). Use a vessel on
the +-5mg scale; tare before each round. The script prompts for the measured
mass (in grams) after each dispense. Acid density ~1.000 g/mL at 0.024 M.
"""

import asyncio
import json
import os
import sys
from datetime import datetime

import websockets

HOST = "khpro.local"
WS_URL = f"ws://{HOST}/ws"
RESULTS_PATH = os.path.join(os.path.dirname(__file__), "results.json")

DISPENSE_UNITS = 7500  # must match DISPENSE_TEST_UNITS in firmware

# (label, command, expected duration)
SCHEDULE = [
    ("fast",    "dispfast",     "~1 min"),
    ("gran200", "dispgran",     "~2 min"),
    ("gran7s",  "dispgranslow", "~18 min"),
    ("gran7s",  "dispgranslow", "~18 min"),
    ("gran7s",  "dispgranslow", "~18 min"),
]

DONE_MARKER = "Dispense test done"
FAIL_MARKERS = ("timeout during dispense test", "Dispense test aborted")
TEST_TIMEOUT_S = 30 * 60


def log(msg: str) -> None:
    ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    print(f"[{ts}] {msg}", flush=True)


async def run_dispense(cmd: str) -> str:
    """Send the command, watch WS messages until done/fail. Returns final text."""
    async with websockets.connect(WS_URL, ping_interval=20) as ws:
        await ws.send(json.dumps({"type": "cmd", "cmd": cmd}))
        log(f"sent cmd '{cmd}', waiting for completion...")
        loop = asyncio.get_event_loop()
        deadline = loop.time() + TEST_TIMEOUT_S
        while loop.time() < deadline:
            try:
                raw = await asyncio.wait_for(ws.recv(), timeout=60)
            except asyncio.TimeoutError:
                continue
            try:
                d = json.loads(raw)
            except (ValueError, TypeError):
                continue
            text = d.get("text") or d.get("message") or ""
            if not text and d.get("type") in ("msg", "error"):
                text = d.get("msg", "")
            if text:
                log(f"  device: {text}")
                if DONE_MARKER in text:
                    return text
                if any(m in text for m in FAIL_MARKERS):
                    raise RuntimeError(f"dispense failed: {text}")
        raise RuntimeError("timed out waiting for dispense completion")


def prompt_mass(label: str) -> float:
    while True:
        try:
            val = input(f"  measured mass for {label} in grams (e.g. 4.921): ").strip()
            return float(val.replace(",", "."))
        except ValueError:
            print("  could not parse — enter a number in grams")


def main() -> None:
    results = []
    print(__doc__)
    log(f"schedule: {[s[0] for s in SCHEDULE]}")

    for i, (label, cmd, duration) in enumerate(SCHEDULE, 1):
        print()
        input(f"[{i}/{len(SCHEDULE)}] {label} ({duration}): place a DRY tared "
              f"vessel under the titration tip, then press Enter...")
        final = asyncio.get_event_loop().run_until_complete(run_dispense(cmd))
        mass = prompt_mass(label)
        ul_per_unit = mass * 1e6 / 1e3 / DISPENSE_UNITS  # g -> uL (rho~1.0) / units
        results.append({
            "label": label, "cmd": cmd, "ts": datetime.now().isoformat(),
            "mass_g": mass, "units": DISPENSE_UNITS,
            "ul_per_unit": round(ul_per_unit, 5), "device_msg": final,
        })
        with open(RESULTS_PATH, "w") as f:
            json.dump(results, f, indent=2)
        log(f"{label}: {mass:.3f} g -> {ul_per_unit:.4f} uL/unit (saved)")

    # --- Verdict ---
    print("\n=== RESULTS ===")
    by = {}
    for r in results:
        by.setdefault(r["label"], []).append(r["ul_per_unit"])
    for label, vals in by.items():
        mean = sum(vals) / len(vals)
        spread = (max(vals) - min(vals)) / mean * 100 if len(vals) > 1 else 0
        print(f"  {label:8s}: {mean:.4f} uL/unit"
              + (f"  (n={len(vals)}, spread {spread:.2f}%)" if len(vals) > 1 else ""))
    if "fast" in by and "gran200" in by:
        d = (sum(by['gran200'])/len(by['gran200']) / (sum(by['fast'])/len(by['fast'])) - 1) * 100
        print(f"  gran200 vs fast: {d:+.2f}%  (|d|>0.5% -> speed-dependent slip)")
    if "gran200" in by and "gran7s" in by:
        d = (sum(by['gran7s'])/len(by['gran7s']) / (sum(by['gran200'])/len(by['gran200'])) - 1) * 100
        print(f"  gran7s vs gran200: {d:+.2f}%  (|d|>0.5% -> dwell-time effect: bubbles/thermal)")
    print(f"\nresults saved to {RESULTS_PATH}")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit("\ninterrupted — partial results saved")
