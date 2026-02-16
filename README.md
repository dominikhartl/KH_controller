# KH Controller

![KH Controller](docs/images/KHcontroller.jpeg)

An ESP32-based automated carbonate hardness (KH) and pH measurement device for reef and freshwater aquariums. It performs acid-base titration to measure alkalinity with high accuracy and integrates seamlessly with Home Assistant.

When I built the first prototype, I made an explanatory video (in English):
[YouTube: KH Controller explained](https://youtu.be/9Bjq2lXnfBI?si=s0VGVHdJ8H3_aLlS)

## How It Works

The device takes a water sample from the aquarium and titrates it with dilute hydrochloric acid (HCl). The volume of acid needed to lower the pH to 4.3 determines the carbonate hardness. The firmware uses a two-phase titration approach:

1. **Fast phase** - Dispenses acid in 200-drop batches until approaching the endpoint pH
2. **Precise phase** - Adaptive 2-4 drop steps with high-precision pH measurement and endpoint interpolation

### Why Is It Accurate?

- **Low acid concentration** (0.02 mol/L HCl): Errors from air bubbles or small dosing variations have minimal impact
- **Pressure titration**: Peristaltic pumps can dispense sub-drop volumes for fine control
- **Adaptive measurement**: More readings and longer mixing time near the endpoint
- **Outlier rejection**: Median-filtered pH readings with automatic outlier removal
- **3-point pH calibration**: Linear least-squares fit across pH 4, 7, and 10 buffer solutions

## Features

- Automated KH measurement via acid-base titration
- Real-time pH monitoring with 3-point calibration
- **Home Assistant integration** via MQTT auto-discovery
- **Web dashboard** with live titration charts, gauges, and configuration
- Scheduled measurements (up to 8 daily time slots, NTP-synced)
- HCl consumption tracking with low-level warnings
- Over-the-air (OTA) firmware updates
- Persistent configuration stored in NVS (flash)

## Hardware

### Bill of Materials

| Component | Quantity | Notes |
|-----------|----------|-------|
| Wemos D1 Mini ESP32 | 1 | Main controller |
| DF-Robot pH Meter V2 | 1 | Analog pH probe |
| NEMA 17 stepper motor (42Ncm, 1.5A) | 2 | For peristaltic pumps |
| TMC2208 stepper driver | 2 | Silent stepper drivers |
| NEMA 17 peristaltic pump | 2 | Sample and titration pumps |
| LM2596S buck converter | 1 | Set to 5V output |
| TIP120 transistor | 1 | Stirrer motor driver |
| 2.2k resistor | 1 | TIP120 base resistor |
| 100uF capacitor | 2 | Power filtering |
| 4.7uF capacitor | 1 | Power filtering |
| 60mm PC cooling fan | 1 | Used as magnetic stirrer motor |
| Magnetic stirrer bar (~1cm) | 1 | Placed in beaker |
| Flat magnets (~1cm, 2mm height) | 2 | Attached to fan |
| 100 mL beaker | 1 | Measurement chamber |
| Silicone tubing (2/4 or 4/6mm) | - | For peristaltic pumps |
| 12V power supply (>2A) | 1 | Main power |
| JST-XH connectors (2/3/4-pin) | - | PCB connections |
| 0.02M HCl solution | - | See preparation below |

### Preparing 0.02M HCl

Mix 10 mL of 37% hydrochloric acid into 5 L of reverse osmosis water (**always add acid to water, never the reverse**). This gives a good balance between accuracy and practical handling volumes.

### PCB Layout

![PCB Layout](docs/images/pcb_layout.png)

The PCB connects the ESP32, pH meter, stepper drivers, stirrer, and all peripherals. Component positions are labeled on the board. Gerber files for manufacturing are in the `PCB/` directory.

### Pin Configuration

| Function | GPIO | Notes |
|----------|------|-------|
| Sample pump ENABLE | 25 | Stepper motor 1 |
| Sample pump DIR | 32 | Stepper motor 1 |
| Sample pump STEP | 2 | Stepper motor 1 |
| Titration pump ENABLE | 22 | Stepper motor 2 |
| Titration pump DIR | 27 | Stepper motor 2 |
| Titration pump STEP | 4 | Stepper motor 2 |
| Stirrer motor | 16 | PWM via TIP120 |
| pH sensor (ADC) | 35 | Analog input |

### 3D Printed Parts

The enclosure consists of three 3D-printed parts (STL files in `3dPrints/`):

- **Corpus.stl** - Main enclosure body
- **ControllerLid.stl** - Controller compartment lid
- **StirrLid.stl** - Stirrer compartment lid

## Firmware

The firmware is a PlatformIO project in the `Firmware/` directory. See the [Firmware documentation](Firmware/README.md) for build instructions, configuration, web interface, Home Assistant integration, MQTT topics, and architecture details.

## Repository Structure

```text
KH_controller/
├── Firmware/          ESP32 firmware (PlatformIO project)
│   ├── src/           Source code (modular C++ architecture)
│   ├── include/       Configuration and pin definitions
│   ├── data/www/      Web dashboard (HTML, CSS, JavaScript)
│   └── README.md      Firmware documentation
├── PCB/               PCB design files (Gerber + layout)
├── 3dPrints/          3D-printable enclosure parts (STL)
├── docs/images/       Documentation images
└── LICENSE            MIT License
```

## License

This project is licensed under the MIT License. See [LICENSE](LICENSE) for details.
