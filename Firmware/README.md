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

   The device advertises itself as `khcontrollerv3.local`. Use the `[env:ota]` environment in `platformio.ini`:

   ```bash
   pio run -e ota -t upload
   ```

### Configuration

All parameters can be configured via the web interface or Home Assistant. They are persisted in flash (NVS) and survive reboots.

| Parameter | Default | Description |
|-----------|---------|-------------|
| Titration volume | 13.4 mL | Volume dispensed per `cal_drops` drops |
| Sample volume | 82.0 mL | Water sample size |
| Correction factor | 1.0 | Manual adjustment multiplier |
| HCl molarity | 0.02 mol/L | Concentration of titration acid |
| HCl volume | 5000 mL | Remaining acid supply |
| Calibration drops | 6000 | Drops counted during pump calibration |
| Fast titration pH | 5.0 | pH threshold for switching to precise mode |
| Endpoint method | Gran (0) | `0` = Gran analysis, `1` = fixed pH endpoint |
| Min start pH | 7.5 | Reject measurement if sample pH is below this |
| Stabilization timeout | 2000 ms | Max wait time for pH reading to stabilize |

## Web Interface

Access the dashboard at `http://khcontrollerv3.local` (or the device's IP address).

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

## Home Assistant Integration

The device uses MQTT auto-discovery, so entities appear automatically in Home Assistant once connected to the same MQTT broker.

### Entities Created

**Sensors**: KH value (dKH), pH (start), measured pH (live), KH trend (dKH/day), measurement confidence, Gran R², cross-validation diff, data points, measurement time, WiFi signal, uptime, probe health, acid slope efficiency, alkaline slope efficiency, probe asymmetry, probe response time, calibration age

**Number inputs** (configurable): Titration volume, sample volume, correction factor, HCl molarity, HCl volume, calibration units, fast titration pH, min start pH, stabilization timeout

**Select inputs**: Endpoint method (Gran/Fixed pH), schedule mode (Custom/Interval), interval hours

**Text inputs**: 8 schedule slots (HH:MM format), anchor time

**Buttons**: Measure KH, Measure pH, Measure Sample, Measure Titration, Fill Titration, Calibrate pH 4/7/10, Measure Voltage, Restart

**Binary sensor**: Device connectivity

## MQTT Topics

All topics are prefixed with `KHcontrollerV3/`:

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
| `gran_r2` | Publish | Gran analysis R² value (retained) |
| `cross_val` | Publish | Cross-validation difference in dKH (retained) |
| `data_pts` | Publish | Number of data points used (retained) |
| `meas_time` | Publish | Measurement duration in seconds (retained) |
| `availability` | Publish | Online/offline status (LWT) |
| `diagnostics` | Publish | RSSI, uptime, free heap, probe health (JSON) |
| `config/*/set` | Subscribe | Configuration commands from HA |
| `config/*` | Publish | Configuration state for HA |

## Commands

Commands can be sent via MQTT (`KHcontrollerV3/cmd`), the web interface, or Home Assistant buttons.

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
| `src/measurement.cpp` | pH/voltage ADC, 3-point calibration, probe health, Gran analysis |
| `src/motors.cpp` | Stepper motor control with acceleration ramps |
| `src/config_store.cpp` | NVS persistent configuration |
| `src/scheduler.cpp` | NTP-based scheduled measurements |
| `src/mqtt_manager.cpp` | MQTT connection with LWT and exponential backoff |
| `src/wifi_manager.cpp` | Non-blocking WiFi state machine |
| `src/stirrer.cpp` | PWM stirrer motor control |
| `include/config.h` | All tuning parameters and constants |
| `include/pins.h` | GPIO pin assignments |
| `data/www/` | Web dashboard (HTML, CSS, JavaScript, Chart.js) |

## License

This project is licensed under the MIT License. See [LICENSE](../LICENSE) for details.
