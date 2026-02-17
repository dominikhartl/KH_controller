#ifndef CONFIG_H
#define CONFIG_H

#include <credentials.h>

// MQTT server and connection
static const char* mqtt_server = "homeassistant.local";
static const int mqtt_port = 1883;

// Device name used as MQTT topic prefix / hostname
static const char DEVICE_NAME[] = "KHcontrollerV3";

// ADC configuration (full-precision mode)
#define ADC_OVERSAMPLING 32
#define ADC_INTER_SAMPLE_DELAY_MS 2

// ADC fast mode (used far from endpoint where ±0.3 pH accuracy suffices)
#define ADC_OVERSAMPLING_FAST 8
#define ADC_INTER_SAMPLE_DELAY_FAST_MS 1
static const int MEASUREMENT_DELAY_FAST_MS = 20;

// pH stabilization (adaptive: waits for readings to converge)
static const int STABILIZATION_TIMEOUT_MS = 2000;
static const float STABILIZATION_THRESHOLD_MV = 3.0;

// pH measurement outlier thresholds
static const float PH_OUTLIER_THRESHOLD = 0.2f;      // Precise mode: ±0.2 pH from median
static const float PH_FAST_OUTLIER_THRESHOLD = 0.3f;  // Fast mode: ±0.3 pH from median

// Motor configuration
static const int STEPS_PER_REVOLUTION = 1600;

// Titration tuning parameters
static const int TITRATION_STEP_SIZE = 2;        // Base units per titration step
static const int MOTOR_STEPS_PER_DROP = 16;      // Motor steps per drop
static const int TITRATION_SPEED = 400;
static const int TITRATION_MIX_DELAY_MS = 500;   // Mixing delay near endpoint
static const int TITRATION_MIX_DELAY_FAST_MS = 200;  // Mixing delay far from endpoint
static const int MAX_TITRATION_UNITS = 10000;
static const int FILL_VOLUME = 100;
static const int PREFILL_SPEED = 300;
static const float MOTOR_TARGET_SPEED = 200;     // Target speed for acceleration ramps
static const float MOTOR_ACCEL_FACTOR = 0.9995;  // Acceleration/deceleration factor
static const int STIRRER_SPEED = 230;
static const int STIRRER_WARMUP_MS = 3000;
static const int MEASUREMENT_DELAY_MS = 50;

// Measurement defaults
static const float ENDPOINT_PH = 4.3f;           // Titration endpoint pH
static const int SAMPLE_PUMP_VOLUME = 350;
static const int CALIBRATION_TARGET_DROPS = 6000;
static const float FAST_TITRATION_PH_DEFAULT = 5.8f; // pH threshold: fast→precise titration

// Motor timing
static const int MOTOR_ENABLE_DELAY_MS = 10;     // Settle time after enabling driver
static const int MOTOR_HOLD_MS = 50;             // Hold position before disabling
static const float MOTOR_START_SPEED = 2000.0;   // Starting speed for acceleration (us)
static const int TITRATE_ACCEL_THRESHOLD = 50;    // Titrate uses acceleration above this volume
static const uint16_t MOTOR_YIELD_INTERVAL = 10; // Yield every N revolutions during long ops

// Motor safety timeouts
static const unsigned long TITRATION_TIMEOUT_MS = 180000;
static const unsigned long SAMPLE_PUMP_TIMEOUT_MS = 600000;

// HCl tracking
static const float HCL_LOW_THRESHOLD_ML = 300.0;

// Gran transformation endpoint detection
static const float GRAN_REGION_PH       = 4.35f;  // Points below this used for Gran regression
static const float GRAN_STOP_PH         = 3.6f;   // Stop titrating at this pH
static const int   MIN_GRAN_POINTS      = 6;       // Minimum points for reliable regression
static const int   MAX_GRAN_POINTS      = 15;      // Stop collecting after this many
static const int   MAX_TITRATION_POINTS = 200;      // Data point buffer size

// Nernst equation: slope(T) = NERNST_FACTOR * T(K) mV/pH
// R*ln(10)/(n*F) = 8.31446 * 2.30259 / (1 * 96485.3) = 0.19842 mV/(pH·K)
static const float NERNST_FACTOR = 0.19842f;
static const float MEASUREMENT_TEMP_C = 21.0f;

// Signal conditioning amplifier gain (hardware constant)
// Derived from acid-side default calibration: (1812-1292) / (3 * 59.16) = 2.93
// Adjust this if your amplifier circuit has a different gain
static const float PH_AMP_GAIN = 2.93f;

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
