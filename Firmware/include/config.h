#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>
#include <credentials.h>

// MQTT server and connection
static const char* mqtt_server = "homeassistant.local";
static const int mqtt_port = 1883;

// Default device name (used as MQTT topic prefix / hostname)
// Runtime name is stored in NVS and loaded into global `deviceName` at boot
static const char DEFAULT_DEVICE_NAME[] = "KHpro";
static const char FW_VERSION[] = "0.5";
// Build timestamp — the only reliable way to tell WHICH build is deployed
// (FW_VERSION is rarely bumped). Shown in /api/diagnostics and the boot message.
static const char FW_BUILD[] = __DATE__ " " __TIME__;

// ADC configuration (full-precision mode)
#define ADC_OVERSAMPLING 64
#define ADC_INTER_SAMPLE_DELAY_MS 10

// ADC fast mode (used far from endpoint where ±0.3 pH accuracy suffices)
#define ADC_OVERSAMPLING_FAST 8
#define ADC_INTER_SAMPLE_DELAY_FAST_MS 1
static const int MEASUREMENT_DELAY_FAST_MS = 20;

// Extra mix/settle delay applied only to the FIRST Gran-zone dose. The probe is
// still settling from the fast/medium-phase exit, so this extra time lets the
// first Gran data point be read on a properly settled solution. (Probe noise is
// captured only at start-pH, so this no longer affects probe_noise_mv.)
static const int GRAN_FIRST_DOSE_EXTRA_MIX_MS = 5000;

// pH stabilization (adaptive: waits for readings to converge)
static const int STABILIZATION_TIMEOUT_MS = 4000;
static const float STABILIZATION_THRESHOLD_MV = 3.0;
static const int STAB_CONSEC_REQUIRED = 3;  // Consecutive reading pairs within threshold before convergence

// pH measurement outlier thresholds
static const float PH_OUTLIER_THRESHOLD = 0.2f;      // Precise mode: ±0.2 pH from median
static const float PH_FAST_OUTLIER_THRESHOLD = 0.3f;  // Fast mode: ±0.3 pH from median
static const float VOLTAGE_OUTLIER_THRESHOLD = 30.0f; // ±30 mV from median (~0.2 pH equivalent)

// KH measurement outlier validation (trend-based)
static const int KH_OUTLIER_HISTORY_COUNT = 28;          // 7 days of history for trend + diurnal prediction
static const int KH_DIURNAL_MIN_POINTS = 8;              // Need 2+ days before fitting diurnal model
static const float KH_OUTLIER_MIN_THRESHOLD = 0.3f;     // Floor: never tighter than this (dKH)
static const float KH_OUTLIER_SIGMA_MULT = 3.0f;        // Adaptive: 3× residual scatter
static const float KH_OUTLIER_FALLBACK_THRESHOLD = 0.5f; // Flat threshold when <3 history points
static const float CROSS_VALIDATION_THRESHOLD_DKH = 0.3f; // Re-measure if Gran vs Endpoint disagree by more

// Motor configuration
static const int STEPS_PER_REVOLUTION = 1600;

// Motor speeds (RPM) — all stepper speeds defined here, converted to us internally
static const float MOTOR_TARGET_RPM   = 80.0f;   // Sample pump cruising speed
static const float MOTOR_START_RPM    = 9.4f;     // Acceleration ramp start speed (~2000 us)
static const float TITRATION_RPM      = 47.0f;    // Titration pump speed in RPM (~400 us)
static const float GRAN_BURST_RPM     = 250.0f;   // Gran zone burst ejection speed (RPM)
static const uint32_t GRAN_BURST_ACCEL = 200000;  // Gran zone burst acceleration (steps/s²)
static const float FILL_BURST_UL  = 50.0f;    // Phase-1 bubble-detach burst volume (µL)
static const float FILL_PULSE_UL  = 1000.0f;  // Phase-2 bubble-flush pulse volume (µL)
static const int   FILL_BURST_COUNT = 10;     // Default phase-1 detach burst count
static const int   FILL_PULSE_COUNT = 5;      // Default phase-2 flush pulse count

// Titration tuning parameters
static const int MOTOR_STEPS_PER_UNIT = 16;      // Motor steps per titration unit
static const int TITRATION_MIX_DELAY_FAST_MS = 200;  // Mixing delay far from endpoint
static const int TITRATION_MIX_DELAY_MEDIUM_MS = 1000; // Pump-calibration medium phase pacing
static const int TITRATION_MIX_DELAY_GRAN_MS = 3500;   // Gran zone mixing (explicit stabilization follows)
static const int MAX_TITRATION_UNITS = 50000;  // Absolute hard cap (user sets soft limit via max_acid_ml)
static const int STIRRER_SPEED_PCT = 90;           // Stirrer duty cycle (%)
static const float SAMPLE_MAX_RPM = 250.0f;        // Sample pump max speed (RPM)
static const float TITRATE_MAX_RPM = 300.0f;       // Titration pump max speed (RPM)
static const int STIRRER_WARMUP_MS = 3000;
static const int PROBE_SETTLE_MS = 30000;  // Extra settle time before start pH (probe transitions ~5 pH units)
static const int MEASUREMENT_DELAY_MS = 50;

// Measurement defaults
static const float ENDPOINT_PH = 4.5f;            // Fixed endpoint titration pH
static const float FIXED_ENDPOINT_STOP_PH = 4.0f; // Stop titrating here in fixed endpoint mode
static const int SAMPLE_CAL_REVOLUTIONS = 350;    // Revolutions used during sample pump calibration
static const int CALIBRATION_TARGET_UNITS = 6000;
static const float FAST_TITRATION_PH_DEFAULT = 5.0f; // pH threshold: fast→precise titration

// Adaptive fast-phase batch sizing — actual batch limits are derived from the
// fast_step_ul config; this only sets where the ramp-down begins
static const float FAST_RAMP_START_PH = 6.0f;  // Start reducing batch size below this pH

// Motor timing
static const uint32_t MOTOR_ACCEL_STEPS_S2 = 1050;  // FastAccelStepper: steps/s² (geometric 0.999/step ≈ 1050; tune after first test)
static const int MOTOR_ENABLE_DELAY_MS = 10;     // Settle time after enabling driver
static const int MOTOR_HOLD_MS = 150;            // Hold position before disabling (tubing settle)
static const int TITRATE_ACCEL_THRESHOLD = 50;    // Titrate uses acceleration above this volume
static const uint16_t MOTOR_YIELD_INTERVAL = 10; // Yield every N revolutions during long ops

// Anti-suckback: small reverse after titration pump stops to prevent drip
static const int ANTI_SUCKBACK_STEPS = 3;
// Backlash compensation steps on sample pump direction reversal
static const int BACKLASH_COMPENSATION_STEPS = 32;

// Motor safety timeouts
static const unsigned long TITRATION_TIMEOUT_MS = 180000;
static const unsigned long SAMPLE_PUMP_TIMEOUT_MS = 600000;

// HCl tracking
static const float HCL_LOW_THRESHOLD_ML = 300.0;

// Starting pH validation
static const float MIN_START_PH_DEFAULT = 7.5f;    // Minimum acceptable starting pH (saltwater)
static const float CARRYOVER_RETRY_PH = 7.0f;      // Below this: hard error; above but below min: retry
static const float POST_WASH_PH_THRESHOLD = 6.0f;  // Warn if post-wash pH is below this

// Gran slope sanity band, applied to the slope of the SELECTED fit window.
// Beyond the equivalence point dF/dV ≈ acid normality; in the selected window
// (pH ~3.5-4.1) the empirical ratio slope/[HCl] is ≈1.03 on healthy runs
// (activity/sulfate effects offset by sub-pH-4 calibration extrapolation —
// measured 1.03/1.04 on healthy runs, 0.51 on the 2026-06-10 stall run).
// A collapsed ratio means part of the indicated acid never neutralized
// anything — motor stall/lost steps, air in the titration line, or inflow.
// NOTE: this is the windowed slope; a whole-curve fit gives ~0.56 instead.
static const float GRAN_SLOPE_RATIO_MIN = 0.80f;
static const float GRAN_SLOPE_RATIO_MAX = 1.30f;

// Gran transformation endpoint detection
static const float GRAN_REGION_PH       = 5.0f;  // Points below this used for Gran regression
static const float GRAN_STOP_PH         = 3.5f;   // Stop titrating at this pH (standard practice: pH 3.0–3.5)
static const int   MIN_GRAN_POINTS      = 8;       // Minimum points for reliable regression
static const int   MAX_TITRATION_POINTS = 200;      // Data point buffer size
static const float GRAN_MIN_R2          = 0.995f;  // Minimum R² for Gran fit acceptance

// Default buffer pH values at 25°C (as printed on bottle)
// User can override via config; device applies temperature compensation automatically
static const float DEFAULT_BUFFER_PH_4  = 4.0f;
static const float DEFAULT_BUFFER_PH_7  = 7.0f;
static const float DEFAULT_BUFFER_PH_10 = 10.0f;

// Nernst equation: slope(T) = NERNST_FACTOR * T(K) mV/pH
// R*ln(10)/(n*F) = 8.31446 * 2.30259 / (1 * 96485.3) = 0.19842 mV/(pH·K)
static const float NERNST_FACTOR = 0.19842f;
static const float DEFAULT_MEASUREMENT_TEMP_C = 21.0f;

// Signal conditioning amplifier gain (hardware constant)
// DFRobot SEN0161-V2 board gain = 3.0 (confirmed from DFRobot_PH library:
// default pH7=1500mV, pH4=2032mV → 532mV / 3pH / 59.16 Nernst@25°C = 3.0)
static const float PH_AMP_GAIN = 3.0f;

// Probe health thresholds
static const float PROBE_EFFICIENCY_GOOD = 95.0f;  // % — above: Good
static const float PROBE_EFFICIENCY_FAIR = 85.0f;   // % — above: Fair, below: Replace
static const float PROBE_ASYMMETRY_GOOD = 15.0f;   // % — below this: Good
static const float PROBE_ASYMMETRY_FAIR = 25.0f;   // % — below this: Fair, above: Replace
static const unsigned long PROBE_RESPONSE_GOOD_MS  = 500;   // Below: healthy response
static const unsigned long PROBE_RESPONSE_FAIR_MS  = 1500;  // Below: fair, above: slow
static const float PROBE_NOISE_GOOD_MV = 6.0f;    // mV StdDev — below: healthy probe
static const float PROBE_NOISE_FAIR_MV = 10.0f;   // mV StdDev — below: fair, above: noisy
static const int CALIBRATION_AGE_WARNING_DAYS = 30;

// Heap monitoring
static const uint32_t HEAP_WARNING_THRESHOLD  = 40000;  // Warn on serial when free heap drops below 40KB
static const uint32_t HEAP_CRITICAL_THRESHOLD = 20000;  // Skip heavy ops (history JSON) below 20KB
static const uint32_t HEAP_RESTART_THRESHOLD  = 10000;  // Graceful restart if sustained below 10KB for 30s

// Minimum valid Unix timestamp (Nov 2023) — used to detect NTP not yet synced
static const uint32_t MIN_VALID_EPOCH = 1700000000;

// Atlas Scientific EZO pH circuit (I2C, via isolation board)
#define EZO_PH_I2C_ADDR    0x63    // Default I2C address (99 decimal)
#define EZO_READ_DELAY_MS   900    // Processing time for R/RT command
#define EZO_CMD_DELAY_MS    300    // Processing time for non-read commands
#define EZO_CAL_DELAY_MS    900    // Processing time for calibration commands
#define EZO_STAB_THRESHOLD  0.02f  // pH stabilization threshold for EZO readings

// pH sensor type (stored in NVS as "ph_sensor")
#define PH_SENSOR_AUTO      0
#define PH_SENSOR_INTERNAL  1
#define PH_SENSOR_ADS1115   2
#define PH_SENSOR_EZO       3

// ADS1115 external ADC configuration
#define ADS1115_I2C_ADDR               0x48
#define ADS_OVERSAMPLING               16       // 16 samples × 75ms = 1.2s; trimmed mean keeps middle 8
#define ADS_OVERSAMPLING_FAST          4
#define ADS_STAB_SAMPLES               8        // 8 samples × 75ms = 600ms per stab iteration (~6 in 4s timeout)
static const float ADS_STABILIZATION_THRESHOLD_MV = 4.0f;  // Matches effective threshold from pre-fix ADS1115 reporting
static const float ADS_MV_PER_BIT = 0.125f;    // GAIN_ONE: ±4.096V / 32768 = 0.125 mV/bit
static const float ADS_EMA_ALPHA  = 0.3f;      // EMA smoothing factor (0.3 = ~3-sample time constant)

// TMC2209 stepper driver configuration
#define TMC_SERIAL_BAUD     115200
#define TMC_SAMPLE_ADDR     0b00    // MS1=LOW, MS2=LOW → address 0
#define TMC_TITRATE_ADDR    0b01    // MS1=HIGH, MS2=LOW → address 1
static const int TMC_SAMPLE_RMS_MA  = 900;   // Sample pump RMS current (mA)
// Titration: 1100 mA for stall margin on the higher-torque fresh tube (2026-06:
// stalled at 150rpm/150k during measurements at 900 mA). Motor is a 1.2 A-class
// 42-stepper; hold is pinned via TMC_TITRATE_HOLD_MULT so idle heat is unchanged.
static const int TMC_TITRATE_RMS_MA = 1100;
static const float TMC_TITRATE_HOLD_MULT = 0.4f;  // 440 mA hold ≈ previous 450
static const float TMC_R_SENSE      = 0.11f; // Current sense resistor (ohms)
static const int TMC_MICROSTEPS     = 8;     // Must match STEPS_PER_REVOLUTION / 200

// MQTT_MAX_PACKET_SIZE and MQTT_KEEPALIVE are set via build_flags in platformio.ini

#endif // CONFIG_H
