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
static const int TITRATION_STEP_SIZE = 2;        // Base drops per titration step
static const int MOTOR_STEPS_PER_DROP = 16;      // Motor steps per drop
static const int TITRATION_SPEED = 400;
static const int TITRATION_MIX_DELAY_MS = 500;   // Mixing delay near endpoint
static const int TITRATION_MIX_DELAY_FAST_MS = 200;  // Mixing delay far from endpoint
static const int MAX_TITRATION_DROPS = 10000;
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

// MQTT_MAX_PACKET_SIZE and MQTT_KEEPALIVE are set via build_flags in platformio.ini

#endif // CONFIG_H
