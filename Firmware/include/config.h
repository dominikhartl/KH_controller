#ifndef CONFIG_H
#define CONFIG_H

#include <credentials.h>

// MQTT server and connection
static const char* mqtt_server = "homeassistant.local";
static const int mqtt_port = 1883;

// Default device name (used as MQTT topic prefix / hostname)
// Runtime name is stored in NVS and loaded into global `deviceName` at boot
static const char DEFAULT_DEVICE_NAME[] = "KHpro";
static const char FW_VERSION[] = "0.4";

// ADC configuration (full-precision mode)
#define ADC_OVERSAMPLING 64
#define ADC_INTER_SAMPLE_DELAY_MS 10

// ADC fast mode (used far from endpoint where ±0.3 pH accuracy suffices)
#define ADC_OVERSAMPLING_FAST 8
#define ADC_INTER_SAMPLE_DELAY_FAST_MS 1
static const int MEASUREMENT_DELAY_FAST_MS = 20;

// pH stabilization (adaptive: waits for readings to converge)
static const int STABILIZATION_TIMEOUT_MS = 4000;
static const float STABILIZATION_THRESHOLD_MV = 3.0;
static const int STAB_CONSEC_REQUIRED = 3;  // Consecutive reading pairs within threshold before convergence

// pH measurement outlier thresholds
static const float PH_OUTLIER_THRESHOLD = 0.2f;      // Precise mode: ±0.2 pH from median
static const float PH_FAST_OUTLIER_THRESHOLD = 0.3f;  // Fast mode: ±0.3 pH from median
static const float VOLTAGE_OUTLIER_THRESHOLD = 30.0f; // ±30 mV from median (~0.2 pH equivalent)

// KH measurement outlier validation
static const float KH_OUTLIER_THRESHOLD_DKH = 0.5f;   // Re-measure if deviation from median exceeds this
static const int KH_OUTLIER_HISTORY_COUNT = 5;         // Number of recent measurements for median
static const float CROSS_VALIDATION_THRESHOLD_DKH = 0.3f; // Re-measure if Gran vs Endpoint disagree by more

// Motor configuration
static const int STEPS_PER_REVOLUTION = 1600;

// Convert RPM to stepper half-period in microseconds
// 1 rev = STEPS_PER_REVOLUTION steps, each step = 2 half-periods
// half_period_us = 60e6 / (2 * RPM * STEPS_PER_REVOLUTION) = 18750 / RPM
inline float rpmToHalfPeriodUs(float rpm) { return 18750.0f / rpm; }

// Motor speeds (RPM) — all stepper speeds defined here, converted to us internally
static const float MOTOR_TARGET_RPM   = 80.0f;   // Sample pump cruising speed
static const float MOTOR_START_RPM    = 9.4f;     // Acceleration ramp start speed (~2000 us)
static const float TITRATION_RPM      = 47.0f;    // Titration pump speed (~400 us)
static const float PREFILL_RPM        = 63.0f;    // Prefill/fill pump speed (~300 us)
static const float MOTOR_ACCEL_FACTOR = 0.9995;   // Acceleration/deceleration factor (per step)

// Titration tuning parameters
static const int TITRATION_STEP_SIZE = 2;        // Base units per titration step
static const int MOTOR_STEPS_PER_UNIT = 16;      // Motor steps per titration unit
static const int TITRATION_MIX_DELAY_FAST_MS = 200;  // Mixing delay far from endpoint
static const int TITRATION_MIX_DELAY_MEDIUM_MS = 1000; // Medium zone mixing (stabilization inside measurePH)
static const int TITRATION_MIX_DELAY_GRAN_MS = 2500;   // Gran zone mixing (explicit stabilization follows)
static const int MAX_TITRATION_UNITS = 10000;
static const int FILL_VOLUME = 100;
// Stirrer speed is configured via configStore (80-100%, default 90%)
static const int STIRRER_WARMUP_MS = 3000;
static const int MEASUREMENT_DELAY_MS = 50;

// Measurement defaults
static const float ENDPOINT_PH = 4.5f;            // Fixed endpoint titration pH
static const float FIXED_ENDPOINT_STOP_PH = 4.0f; // Stop titrating here in fixed endpoint mode
static const int SAMPLE_PUMP_VOLUME = 350;        // Legacy: use configStore.getSampleCalRevsPerML() * getSampleVolume()
static const int SAMPLE_CAL_REVOLUTIONS = 350;    // Revolutions used during sample pump calibration
static const int CALIBRATION_TARGET_UNITS = 6000;
static const float FAST_TITRATION_PH_DEFAULT = 5.0f; // pH threshold: fast→precise titration

// Medium zone step multiplier (TITRATION_STEP_SIZE * this = units per medium step)
// Smaller steps yield more data points for Gran regression
static const int MEDIUM_STEP_MULTIPLIER = 12;  // 2 * 12 = 24 units per step

// Gran zone step multiplier — smaller = more data points for regression robustness
static const int GRAN_STEP_MULTIPLIER = 8;     // 2 * 8 = 16 units per step

// Adaptive fast-phase batch sizing — reduces batch as pH approaches threshold
static const int FAST_BATCH_MAX = 200;
static const int FAST_BATCH_MIN = 20;
static const float FAST_RAMP_START_PH = 6.0f;  // Start reducing batch size below this pH

// Motor timing
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

// Gran transformation endpoint detection
static const float GRAN_REGION_PH       = 5.0f;  // Points below this used for Gran regression
static const float GRAN_STOP_PH         = 3.5f;   // Stop titrating at this pH (literature: 3.0-4.5 optimal)
static const int   MIN_GRAN_POINTS      = 8;       // Minimum points for reliable regression
static const int   MAX_TITRATION_POINTS = 200;      // Data point buffer size
static const float GRAN_MIN_R2          = 0.99f;   // Minimum R² for Gran fit acceptance

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
static const uint32_t HEAP_WARNING_THRESHOLD = 40000;  // Warn on serial when free heap drops below 40KB

// Minimum valid Unix timestamp (Nov 2023) — used to detect NTP not yet synced
static const uint32_t MIN_VALID_EPOCH = 1700000000;

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
static const int TMC_SAMPLE_RMS_MA  = 800;   // Sample pump RMS current (mA)
static const int TMC_TITRATE_RMS_MA = 600;   // Titration pump RMS current (mA)
static const float TMC_R_SENSE      = 0.11f; // Current sense resistor (ohms)
static const int TMC_MICROSTEPS     = 8;     // Must match STEPS_PER_REVOLUTION / 200
static const int TMC_STALL_THRESHOLD = 15;   // SGTHRS: DIAG triggers at SG ≤ SGTHRS*2 (=30)
static const int TMC_SAMPLE_STALL_SG = 30;  // Sample pump SG stall threshold (normal ~96-258)

// MQTT_MAX_PACKET_SIZE and MQTT_KEEPALIVE are set via build_flags in platformio.ini

#endif // CONFIG_H
