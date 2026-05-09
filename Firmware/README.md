# KH Controller - Firmware

![KH Controller](../docs/images/KHcontroller.jpeg)

An ESP32-based automated carbonate hardness (KH) and pH measurement device for reef and freshwater aquariums. It performs acid-base titration to measure alkalinity with high accuracy and integrates seamlessly with Home Assistant.

For hardware details (bill of materials, PCB layout, pin configuration, HCl preparation, 3D prints), see the [main README](../README.md).

## Getting Started

### Prerequisites

- [PlatformIO](https://platformio.org/) (VS Code extension or CLI)
- An MQTT broker (e.g., Mosquitto, or the one built into Home Assistant)
- WiFi network

### Setup

1. **Clone the repository**

   ```bash
   git clone https://github.com/dominikhartl/KH_controller.git
   cd KH_controller/Firmware
   ```

2. **Configure credentials**

   ```bash
   cp include/credentials.h.example include/credentials.h
   ```

   Edit `include/credentials.h` with your WiFi and MQTT credentials.

3. **Build and upload firmware** (via USB)

   ```bash
   pio run -t upload
   ```

4. **Upload web interface** (LittleFS filesystem)

   ```bash
   pio run -t uploadfs
   ```

5. **OTA updates** (after initial USB upload)

   The device advertises itself via mDNS (default: `khpro.local`). Use the `[env:ota]` environment:

   ```bash
   pio run -e ota -t upload                                    # default device
   pio run -e ota -t upload --upload-port mydevice.local        # specific device
   ```

### pH Sensor Options

The firmware supports three pH sensor configurations, selectable in **Settings → Advanced → pH Sensor**:

| Sensor | Type | Resolution | Notes |
|--------|------|------------|-------|
| DF-Robot pH Meter V2 | Analog (ESP32 internal ADC) | 12-bit | Default, no extra hardware needed |
| DF-Robot + ADS1115 | Analog (external 16-bit ADC) | 16-bit | Lower noise, I2C on GPIO26/33 |
| Atlas Scientific EZO pH | Digital (I2C) | ±0.002 pH | Requires EZO circuit + isolation board |

**Auto-detection (default):** On boot, the firmware probes I2C in priority order: EZO (0x63) → ADS1115 (0x48) → internal ADC. The first detected device is used. Override this via the pH Sensor setting if needed.

**EZO pH calibration:**
- The EZO circuit stores calibration data onboard — it persists even without power
- Calibration must be done in order: **pH 7 first**, then pH 4, then pH 10 (issuing pH 7 clears previous calibration)
- The web UI enforces this order by disabling buttons until the prerequisite step is complete
- At least a 2-point calibration (pH 7 + pH 4) is required for measurements
- Probe health is reported via the EZO's `SLOPE,?` command (acid/base slope percentages)

**Switching sensors:** Recalibrate pH after switching sensor type — calibration data is sensor-specific.

### Multi-Device Setup

Multiple devices can run on the same network. Each device needs a unique name:

1. Open the web UI and set **Device Name** in Configuration (e.g., `KHpro-reef`, `KHpro-frag`)
2. Reboot the device — mDNS hostname, MQTT topics, and HA discovery IDs update automatically
3. Upload to a specific device using `--upload-port`:

   ```bash
   pio run -e ota -t upload --upload-port khpro-reef.local
   pio run -e ota -t uploadfs --upload-port khpro-reef.local
   ```

### Configuration

All parameters can be configured via the web interface or Home Assistant. They are persisted in flash (NVS) and survive reboots.

| Parameter | Default | Description |
|-----------|---------|-------------|
| Device name | KHpro | mDNS hostname, MQTT prefix, HA device ID (reboot to apply) |
| Titration volume | 13.4 mL | Volume dispensed per `cal_drops` drops |
| Correction factor | 1.0 | Manual adjustment multiplier |
| HCl molarity | 0.02 mol/L | Concentration of titration acid |
| HCl volume | 5000 mL | Remaining acid supply |
| Calibration drops | 6000 | Drops counted during pump calibration |
| Fast titration pH | 5.0 | pH threshold for switching to precise mode |
| Endpoint method | Gran (0) | `0` = Gran analysis, `1` = fixed pH endpoint |
| Min start pH | 7.5 | Reject measurement if sample pH is below this |
| Stabilization timeout | 2000 ms | Max wait time for pH reading to stabilize |
| pH sensor | Auto (0) | `0` = Auto-detect, `1` = Internal ADC, `2` = ADS1115, `3` = EZO pH |

## Web Interface

Access the dashboard at `http://<devicename>.local` (default: `http://khpro.local`) or the device's IP address.

The web interface provides:

- **KH, pH, and measured pH gauges** with last measurement values
- **HCl tank level** indicator showing remaining acid
- **Live titration chart** showing pH vs. volume (mL) with linear x-axis during measurement
- **Gran analysis chart** with scatter plot, regression line, and R² display
- **Gran history chart** tracking R² and endpoint pH across measurements
- **Historical charts** for KH and pH (7-day rolling window) with trend line
- **KH trend** (dKH/day) computed via linear regression
- **Measurement confidence** score combining R², cross-validation, and data quality
- **Progress bar** during active measurements
- **Command buttons** for measurement, calibration, and maintenance
- **Configuration panel** with common settings and collapsible advanced section
- **Schedule editor** with custom time slots or interval mode
- **Probe health** section showing acid/alkaline slope efficiency, asymmetry, calibration age, and efficiency trend sparkline
- **Event log** with timestamped messages and errors
- **Status indicators** for WiFi, MQTT, NTP, WebSocket, and probe health
- **CSV export** of measurement history
- **Hardware diagnostics** with comprehensive ADC noise analysis, I2C bus health, motor tests, temperature sensor validation, GPIO state checks, and downloadable JSON report for troubleshooting and data-driven hardware improvements

## Home Assistant Integration

The device uses MQTT auto-discovery, so entities appear automatically in Home Assistant once connected to the same MQTT broker.

### Entities Created

**Sensors**: KH value (dKH), pH (start), measured pH (live), KH trend (dKH/day), KH deviation (dKH, signed `measured − predicted`), measurement confidence, Gran R², cross-validation diff, data points, measurement time, sample volume (computed from calibration), WiFi signal, uptime, probe health, acid slope efficiency, alkaline slope efficiency, probe asymmetry, probe response time, calibration age

**Number inputs** (configurable): Titration volume, correction factor, HCl molarity, HCl volume, calibration units, fast titration pH, min start pH, stabilization timeout

**Select inputs**: Endpoint method (Gran/Fixed pH), schedule mode (Custom/Interval), interval hours

**Text inputs**: 8 schedule slots (HH:MM format), anchor time

**Buttons**: Measure KH, Measure pH, Measure Sample, Measure Titration, Fill Titration, Calibrate pH 4/7/10, Measure Voltage, Restart

**Binary sensors**: Device connectivity, KH deviation alert (`device_class: problem`)

### KH Deviation Alert

The firmware fits a trend (and a diurnal cycle when ≥8 history points are available) to recent KH measurements and predicts the expected value at the time of each new measurement. If a measurement disagrees with the prediction, a re-measurement is run automatically. The **KH Deviation Alert** binary sensor turns `on` only when even the re-measurement is still outside the expected band — a strong signal of a real change (water change, dosing event, leak, probe drift). It auto-clears on the next in-threshold measurement. The signed **KH Deviation** sensor reports `measured − predicted` in dKH so HA notifications can include direction and magnitude. Both states are retained, so HA preserves the alert across reboots.

Example HA automation:

```yaml
alias: KH Deviation Alert
trigger:
  - platform: state
    entity_id: binary_sensor.khpro_kh_deviation_alert
    to: "on"
action:
  - service: notify.mobile_app_<your_phone>
    data:
      title: "KH deviated from prediction"
      message: >-
        KH is {{ states('sensor.khpro_kh') }} dKH
        ({{ states('sensor.khpro_kh_deviation') }} dKH vs prediction).
```

## MQTT Topics

All topics are prefixed with the device name (default `KHpro/`):

| Topic | Direction | Description |
|-------|-----------|-------------|
| `cmd` | Subscribe | Command input (see commands below) |
| `message` | Publish | Status messages |
| `error` | Publish | Error messages |
| `kh_value` | Publish | Calculated KH in dKH (retained) |
| `startPH` | Publish | pH at start of titration (retained) |
| `mes_pH` | Publish | Live pH during measurement |
| `confidence` | Publish | Measurement confidence score (retained) |
| `kh_slope` | Publish | KH trend in dKH/day (retained) |
| `kh_deviation` | Publish | Signed deviation `measured − predicted` in dKH (retained) |
| `kh_deviation_alert` | Publish | `ALERT` / `OK` — fires when re-measurement still deviates (retained) |
| `gran_r2` | Publish | Gran analysis R² value (retained) |
| `cross_val` | Publish | Cross-validation difference in dKH (retained) |
| `data_pts` | Publish | Number of data points used (retained) |
| `meas_time` | Publish | Measurement duration in seconds (retained) |
| `availability` | Publish | Online/offline status (LWT) |
| `diagnostics` | Publish | RSSI, uptime, free heap, probe health (JSON) |
| `config/*/set` | Subscribe | Configuration commands from HA |
| `config/*` | Publish | Configuration state for HA |

## Commands

Commands can be sent via MQTT (`<devicename>/cmd`), the web interface, or Home Assistant buttons.

| Command | Code | Description |
|---------|------|-------------|
| Measure KH | `k` | Full KH titration measurement |
| Measure pH | `p` | Single pH reading |
| Fill titration | `f` | Prime the titration tube with acid |
| Wash sample | `s` | Flush the sample chamber |
| Calibrate pump | `t` | Run calibration (6000 drops) |
| Start stirrer | `m` | Turn on the magnetic stirrer |
| Stop stirrer | `e` | Turn off the magnetic stirrer |
| Remove sample | `r` | Empty the sample chamber |
| Restart | `o` | Reboot the device |
| Measure voltage | `v` | Raw pH probe voltage reading |
| Motor diagnostics | `d` | Test both motor drivers in StealthChop and SpreadCycle modes |
| Hardware diagnostics | `H` | Run full hardware diagnostic suite (~90s) |
| Calibrate pH 4 | `4` | Calibrate with pH 4 buffer |
| Calibrate pH 7 | `7` | Calibrate with pH 7 buffer |
| Calibrate pH 10 | `10` | Calibrate with pH 10 buffer |

## Calculating KH

The device supports two endpoint detection methods:

### Gran Analysis (default)

Gran analysis uses a linearization technique to find the titration equivalence point without requiring the pH to actually reach 4.3. The Gran function `F = V_acid × 10^(-pH)` is plotted against acid volume; the x-intercept of the linear regression through these points gives the equivalence volume. This method is more robust to probe drift and provides quality metrics (R², confidence).

### Fixed pH Endpoint

Classic approach: interpolate the exact acid volume where pH crosses 4.3 (or configured endpoint pH).

### KH Formula

```
KH [dKH] = (V_acid / sample_vol) * 2800 * hcl_molarity * correction_factor
```

Where `V_acid` is the equivalence volume in mL (derived from drops: `V_acid = drops / cal_drops * titration_vol`).

### Measurement Validation

Each measurement is validated with:
- **Cross-validation**: Compares Gran and fixed-pH results; large discrepancies flag unreliable readings
- **R² threshold**: Gran regression must have R² > 0.95
- **Minimum data points**: At least 3 valid Gran points required
- **Start pH check**: Sample pH must exceed `min_start_ph` (default 7.5)
- **Confidence score**: Combines R², cross-validation difference, and data point count into a single quality metric

For best results, calibrate the pump (`t`) and carefully measure the dispensed volume.

## Architecture

| File | Purpose |
|------|---------|
| `src/main.cpp` | Application entry, KH measurement algorithm, MQTT routing |
| `src/web_server.cpp` | HTTP server, WebSocket dashboard, command dispatch |
| `src/ha_discovery.cpp` | Home Assistant MQTT auto-discovery |
| `src/measurement.cpp` | pH/voltage ADC (internal + ADS1115 + EZO), calibration, probe health, Gran analysis |
| `src/motors.cpp` | Stepper motor control with acceleration ramps |
| `src/config_store.cpp` | NVS persistent configuration |
| `src/scheduler.cpp` | NTP-based scheduled measurements |
| `src/mqtt_manager.cpp` | MQTT connection with LWT and exponential backoff |
| `src/wifi_manager.cpp` | Non-blocking WiFi state machine |
| `src/stirrer.cpp` | PWM stirrer motor control |
| `src/temperature.cpp` | DS18B20 temperature sensor with Nernst compensation |
| `src/hw_diagnostics.cpp` | Comprehensive hardware diagnostics (ADC noise, I2C, motors, GPIO, probe) |
| `src/tmc_driver.cpp` | TMC2209 stepper driver UART interface |
| `include/config.h` | All tuning parameters and constants |
| `include/pins.h` | GPIO pin assignments |
| `data/www/` | Web dashboard (HTML, CSS, JavaScript, Chart.js) |

## Hardware Diagnostics

The device includes a comprehensive hardware diagnostic suite accessible from the web interface under **Configuration > Advanced > Hardware Diagnostics**. Run it by clicking "Run Full Diagnostics" (~90 seconds). The results are downloadable as a JSON report (`/api/hwdiag`).

Tests performed:

- **ADC noise analysis**: Rapid sampling (500 readings) on the pH channel with histogram and time series. Multi-rate noise test across all 8 ADS1115 data rates (8–860 SPS) to characterize noise vs bandwidth tradeoffs. Baseline noise measurement on an unconnected ADS1115 channel (AIN1) to separate circuit noise from inherent ADC noise. Internal ESP32 ADC noise test always runs for comparison.
- **I2C bus health**: Config register readback verification, NAK count, conversion timing, RDY pin check, bus scan.
- **Temperature sensor**: DS18B20 consistency check (5 readings), CRC error and power-on-reset detection.
- **TMC2209 motor drivers**: UART communication test, DRV_STATUS flags (overtemp, open load, short circuit).
- **Motor performance**: StealthChop vs SpreadCycle StallGuard comparison for both pumps.
- **GPIO pin states**: Motor enables, DIAG pins, RDY pin — expected vs actual state.
- **pH probe health**: Nernst efficiency, asymmetry, noise stats, calibration age (from cached data, no new measurement).
- **System health**: Heap usage, uptime, reset reason, flash usage, WiFi RSSI.

All tests gracefully handle missing hardware (ADS1115, TMC2209, DS18B20) — absent components are reported as `"skipped": true` in the JSON.

The noise data is designed for data-driven hardware improvements: the noise ratio (circuit noise / ADC baseline noise) quantifies how much noise the pH probe and signal conditioning board add, informing decisions about capacitors, low-pass filters, and shielding.

## ADS1115 External ADC (Optional)

The firmware supports an optional ADS1115 16-bit external ADC module connected via I2C (SDA=GPIO26, SCL=GPIO33) for higher-resolution pH measurement. The ADS1115 provides 16-bit resolution vs the ESP32's 12-bit internal ADC, resulting in lower noise and more precise readings.

- Auto-detected at boot (or select "ADS1115" in pH Sensor setting)
- Separate calibration values are stored for internal and external ADC — recalibrate pH after switching
- If configured but not detected at boot, the firmware falls back to the internal ADC automatically
- The web dashboard shows current probe mV and calibration voltages in the Probe Health section

## License

This project is licensed under the MIT License. See [LICENSE](../LICENSE) for details.
