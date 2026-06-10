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
import time
import urllib.request
from datetime import datetime

import websockets

# mDNS resolution and weak RSSI make single connect attempts unreliable —
# try hostname first, then the device's last known IP, with retries.
HOSTS = ["khpro.local", "192.168.10.39"]
RESULTS_PATH = os.path.join(os.path.dirname(__file__), "results.json")

DISPENSE_UNITS = 7500  # must match DISPENSE_TEST_UNITS in firmware

# (label, command, expected duration)
SCHEDULE = [
    ("fast",    "dispfast",     "~1 min"),
    ("gran200", "dispgran",     "~2 min"),
    ("gran7s",  "dispgranslow", "~18 min"),
    ("gran7s",  "dispgranslow", "~18 min"),
    ("gran7s",  "dispgranslow", "~18 min"),
    # fast repeat at the END: round 1 paid the dry-tip priming cost (~20-30 uL);
    # this one starts from a wetted line and separates priming from real slip
    ("fast",    "dispfast",     "~1 min"),
]

DONE_MARKER = "Dispense test done"
FAIL_MARKERS = ("timeout during dispense test", "Dispense test aborted")
TEST_TIMEOUT_S = 30 * 60


def log(msg: str) -> None:
    ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    print(f"[{ts}] {msg}", flush=True)


async def ws_connect_with_retry():
    """Try each host up to 3 times with backoff before giving up."""
    last_exc = None
    for attempt in range(3):
        for host in HOSTS:
            try:
                return await websockets.connect(f"ws://{host}/ws",
                                                ping_interval=20, open_timeout=10)
            except Exception as e:  # noqa: BLE001 — retry any connect failure
                last_exc = e
                log(f"connect to {host} failed ({type(e).__name__}), retrying...")
        await asyncio.sleep(5 * (attempt + 1))
    raise RuntimeError(f"could not connect to device: {last_exc}")


def check_event_log(since_epoch: float) -> str:
    """HTTP fallback: scan the device event log for a done/fail message newer
    than since_epoch. Returns the message text or ''."""
    try:
        for host in HOSTS:
            try:
                req = urllib.request.Request(f"http://{host}/api/diagnostics")
                with urllib.request.urlopen(req, timeout=20) as resp:
                    d = json.loads(resp.read())
                break
            except Exception:  # noqa: BLE001
                d = None
        if not d:
            return ""
        for e in d.get("event_log", []):
            if e.get("ts", 0) >= since_epoch - 60:
                text = e.get("text", "")
                if DONE_MARKER in text or any(m in text for m in FAIL_MARKERS):
                    return text
    except Exception:  # noqa: BLE001
        pass
    return ""


async def run_dispense(cmd: str) -> str:
    """Send the command once, then watch until done/fail. The dispense runs on
    the device independently — WS drops (weak WiFi, invalid frames) are
    survived by reconnecting and consulting the device event log for anything
    missed while disconnected."""
    sent = False
    start_epoch = time.time()
    deadline = time.time() + TEST_TIMEOUT_S
    while time.time() < deadline:
        try:
            async with await ws_connect_with_retry() as ws:
                if not sent:
                    await ws.send(json.dumps({"type": "cmd", "cmd": cmd}))
                    sent = True
                    log(f"sent cmd '{cmd}', waiting for completion...")
                else:
                    # Reconnected — did we miss the completion message?
                    text = check_event_log(start_epoch)
                    if text:
                        log(f"  device (from log): {text}")
                        if DONE_MARKER in text:
                            return text
                        raise RuntimeError(f"dispense failed: {text}")
                while time.time() < deadline:
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
        except (websockets.exceptions.ConnectionClosed, OSError) as e:
            log(f"WS dropped ({type(e).__name__}) — reconnecting; "
                f"the dispense continues on the device")
            await asyncio.sleep(3)
    raise RuntimeError("timed out waiting for dispense completion")


def prompt_mass(label: str) -> float:
    while True:
        try:
            val = input(f"  measured mass for {label} in grams (e.g. 4.921): ").strip()
            return float(val.replace(",", "."))
        except ValueError:
            print("  could not parse — enter a number in grams")


def main() -> None:
    # Resume: completed rounds are in results.json — skip them
    results = []
    if os.path.exists(RESULTS_PATH):
        with open(RESULTS_PATH) as f:
            results = json.load(f)
        if results:
            log(f"resuming — {len(results)} round(s) already done: "
                f"{[r['label'] for r in results]}")
    print(__doc__)
    log(f"schedule: {[s[0] for s in SCHEDULE]}")

    for i, (label, cmd, duration) in enumerate(SCHEDULE, 1):
        if i <= len(results):
            continue  # already completed in a previous session
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


def record_manual(label: str, mass_g: float) -> None:
    """Append a round whose dispense completed but whose mass wasn't recorded
    (e.g. the script crashed mid-watch): --record <label> <grams>."""
    results = []
    if os.path.exists(RESULTS_PATH):
        with open(RESULTS_PATH) as f:
            results = json.load(f)
    ul_per_unit = mass_g * 1000.0 / DISPENSE_UNITS
    results.append({
        "label": label, "cmd": "(manual)", "ts": datetime.now().isoformat(),
        "mass_g": mass_g, "units": DISPENSE_UNITS,
        "ul_per_unit": round(ul_per_unit, 5), "device_msg": "recorded manually",
    })
    with open(RESULTS_PATH, "w") as f:
        json.dump(results, f, indent=2)
    log(f"recorded {label}: {mass_g:.3f} g -> {ul_per_unit:.4f} uL/unit "
        f"({len(results)} rounds total)")


if __name__ == "__main__":
    try:
        if len(sys.argv) == 4 and sys.argv[1] == "--record":
            record_manual(sys.argv[2], float(sys.argv[3]))
        else:
            main()
    except KeyboardInterrupt:
        sys.exit("\ninterrupted — partial results saved")
