#ifndef CONFIG_H
#define CONFIG_H

#include <credentials.h>

// MQTT server and connection
static const char* mqtt_server = "homeassistant.local";
static const int mqtt_port = 1883;

// Device name used as MQTT topic prefix / hostname
static const char DEVICE_NAME[] = "KHcontrollerV3";

// ADC configuration (full-precision mode)
#define ADC_OVERSAMPLING 64
#define ADC_INTER_SAMPLE_DELAY_MS 2

// ADC fast mode (used far from endpoint where ±0.3 pH accuracy suffices)
#define ADC_OVERSAMPLING_FAST 8
#define ADC_INTER_SAMPLE_DELAY_FAST_MS 1
static const int MEASUREMENT_DELAY_FAST_MS = 20;

// pH stabilization (adaptive: waits for readings to converge)
static const int STABILIZATION_TIMEOUT_MS = 2000;
static const float STABILIZATION_THRESHOLD_MV = 2.0;

// pH measurement outlier thresholds
static const float PH_OUTLIER_THRESHOLD = 0.2f;      // Precise mode: ±0.2 pH from median
static const float PH_FAST_OUTLIER_THRESHOLD = 0.3f;  // Fast mode: ±0.3 pH from median
static const float VOLTAGE_OUTLIER_THRESHOLD = 30.0f; // ±30 mV from median (~0.2 pH equivalent)

// KH measurement outlier validation
static const float KH_OUTLIER_THRESHOLD_DKH = 1.0f;  // Re-measure if deviation from median exceeds this
static const int KH_OUTLIER_HISTORY_COUNT = 5;        // Number of recent measurements for median

// Motor configuration
static const int STEPS_PER_REVOLUTION = 1600;

// Convert RPM to stepper half-period in microseconds
// 1 rev = STEPS_PER_REVOLUTION steps, each step = 2 half-periods
// half_period_us = 60e6 / (2 * RPM * STEPS_PER_REVOLUTION) = 18750 / RPM
inline float rpmToHalfPeriodUs(float rpm) { return 18750.0f / rpm; }

// Motor speeds (RPM) — all stepper speeds defined here, converted to us internally
static const float MOTOR_TARGET_RPM   = 94.0f;   // Sample pump cruising speed (~200 us)
static const float MOTOR_START_RPM    = 9.4f;     // Acceleration ramp start speed (~2000 us)
static const float TITRATION_RPM      = 47.0f;    // Titration pump speed (~400 us)
static const float PREFILL_RPM        = 63.0f;    // Prefill/fill pump speed (~300 us)
static const float MOTOR_ACCEL_FACTOR = 0.9995;   // Acceleration/deceleration factor (per step)

// Titration tuning parameters
static const int TITRATION_STEP_SIZE = 2;        // Base units per titration step
static const int MOTOR_STEPS_PER_UNIT = 16;      // Motor steps per titration unit
static const int TITRATION_MIX_DELAY_MS = 2000;  // Legacy: full mixing delay (used by calibration)
static const int TITRATION_MIX_DELAY_FAST_MS = 200;  // Mixing delay far from endpoint
static const int TITRATION_MIX_DELAY_MEDIUM_MS = 1000; // Medium zone mixing (stabilization inside measurePH)
static const int TITRATION_MIX_DELAY_GRAN_MS = 1000;   // Gran zone mixing (explicit stabilization follows)
static const int MAX_TITRATION_UNITS = 10000;
static const int FILL_VOLUME = 100;
static const int STIRRER_SPEED = 230;            // PWM duty cycle (0-255), not RPM
static const int STIRRER_WARMUP_MS = 3000;
static const int MEASUREMENT_DELAY_MS = 50;

// Measurement defaults
static const float ENDPOINT_PH = 4.4f;            // Fixed endpoint titration pH
static const float FIXED_ENDPOINT_STOP_PH = 4.0f; // Stop titrating here in fixed endpoint mode
static const int SAMPLE_PUMP_VOLUME = 350;
static const int CALIBRATION_TARGET_UNITS = 6000;
static const float FAST_TITRATION_PH_DEFAULT = 5.0f; // pH threshold: fast→precise titration

// Medium zone step multiplier (TITRATION_STEP_SIZE * this = units per medium step)
// Medium zone points aren't used by Gran analysis, so large steps are fine
static const int MEDIUM_STEP_MULTIPLIER = 24;  // 2 * 24 = 48 units per step

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
static const float GRAN_REGION_PH       = 4.5f;  // Points below this used for Gran regression
static const float GRAN_STOP_PH         = 3.5f;   // Stop titrating at this pH (Dickson protocol)
static const int   MIN_GRAN_POINTS      = 8;       // Minimum points for reliable regression
static const int   MAX_TITRATION_POINTS = 200;      // Data point buffer size
static const float GRAN_MIN_R2          = 0.99f;   // Minimum R² for Gran fit acceptance

// Calibration buffer pH values at measurement temperature
static const float BUFFER_PH_4  = 4.00f;
static const float BUFFER_PH_7  = 7.02f;
static const float BUFFER_PH_10 = 10.07f;

// Nernst equation: slope(T) = NERNST_FACTOR * T(K) mV/pH
// R*ln(10)/(n*F) = 8.31446 * 2.30259 / (1 * 96485.3) = 0.19842 mV/(pH·K)
static const float NERNST_FACTOR = 0.19842f;
static const float MEASUREMENT_TEMP_C = 21.0f;

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
static const int CALIBRATION_AGE_WARNING_DAYS = 30;

// MQTT_MAX_PACKET_SIZE and MQTT_KEEPALIVE are set via build_flags in platformio.ini

#endif // CONFIG_H
