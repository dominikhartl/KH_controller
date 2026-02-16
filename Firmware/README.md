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
| Fast titration pH | 5.8 | pH threshold for switching to precise mode |

## Web Interface

Access the dashboard at `http://khcontrollerv3.local` (or the device's IP address).

The web interface provides:

- **KH and pH gauges** with last measurement values
- **HCl tank level** indicator showing remaining acid
- **Live titration chart** showing pH vs. drops during measurement
- **Historical charts** for KH and pH (7-day rolling window)
- **Command buttons** for measurement, calibration, and maintenance
- **Configuration panel** for all measurement parameters
- **Schedule editor** for automated daily measurements
- **Event log** with timestamped messages and errors
- **Status indicators** for WiFi, MQTT, NTP, and WebSocket connectivity

## Home Assistant Integration

The device uses MQTT auto-discovery, so entities appear automatically in Home Assistant once connected to the same MQTT broker.

### Entities Created

**Sensors**: KH value (dKH), pH, measurement pH (live), WiFi signal, uptime

**Number inputs** (configurable): Titration volume, sample volume, correction factor, HCl molarity, HCl volume, calibration drops, fast titration pH

**Text inputs**: 8 schedule slots (HH:MM format)

**Buttons**: Measure KH, Measure pH, Wash sample, Fill titration, Calibrate pump, Calibrate pH 4/7/10, Measure voltage, Restart

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
| `KH` | Publish | Number of titration drops used |
| `availability` | Publish | Online/offline status (LWT) |
| `diagnostics` | Publish | RSSI, uptime, free heap (JSON) |
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

The device calculates KH using:

```
KH [dKH] = (drops / cal_drops) * titration_vol / sample_vol * 2800 * hcl_molarity * correction_factor
```

- `drops` — interpolated endpoint crossing from titration
- `cal_drops` — drops counted during pump calibration (command `t`)
- `titration_vol` — volume dispensed during calibration (measure accurately!)
- `sample_vol` — water sample volume (measure accurately!)
- `hcl_molarity` — acid concentration (default 0.02 mol/L)
- `correction_factor` — manual adjustment if needed

For best results, calibrate the pump (`t`) and carefully measure the dispensed volume.

## Architecture

| File | Purpose |
|------|---------|
| `src/main.cpp` | Application entry, KH measurement algorithm, MQTT routing |
| `src/web_server.cpp` | HTTP server, WebSocket dashboard, command dispatch |
| `src/ha_discovery.cpp` | Home Assistant MQTT auto-discovery |
| `src/measurement.cpp` | pH/voltage ADC sampling with outlier rejection |
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
