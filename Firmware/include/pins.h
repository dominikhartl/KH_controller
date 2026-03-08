#ifndef PINS_H
#define PINS_H

// Stepper motor 1 - Sample pump
#define EN_PIN1 25
#define DIR_PIN1 32
#define STEP_PIN1 2

// Stepper motor 2 - Titration pump
#define EN_PIN2 22
#define DIR_PIN2 27
#define STEP_PIN2 4

// Stirrer motor
#define STIRRER_PIN 16

// pH sensor (internal ADC fallback — only used when ADS1115 not present)
#define PH_PIN 34

// ADS1115 external ADC (I2C)
#define I2C_SDA_PIN  26
#define I2C_SCL_PIN  33
// ADS1115 RDY pin (GPIO34, shared with PH_PIN — mutually exclusive usage:
// internal ADC mode → analog input, ADS1115 mode → digital RDY polling)
#define ADS_RDY_PIN  34

// TMC2209 UART (shared bus, drivers distinguished by address)
#define TMC_UART_TX    18
#define TMC_UART_RX    19

// TMC2209 DIAG pins (active-high stall indication)
#define DIAG_SAMPLE    23
#define DIAG_TITRATE   35

// DS18B20 temperature sensor (OneWire)
#define TEMP_SENSOR_PIN 5

#endif // PINS_H
