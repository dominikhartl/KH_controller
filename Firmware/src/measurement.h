#ifndef MEASUREMENT_H
#define MEASUREMENT_H

// Measurement state - defined in measurement.cpp
extern float voltage;
extern float pH;
extern float voltage_4PH;
extern float voltage_7PH;
extern float voltage_10PH;

// Initialize ADC with calibrated attenuation
void initADC();

// Recompute linear fit coefficients from current calibration voltages
void updateCalibrationFit();

// Check if calibration voltages are valid (not NaN, sufficient separation)
bool isCalibrationValid();

// Measurement functions
void measurePH(int nreadings);
void measurePHFast(int nreadings);  // No stabilization, 8x oversample — for far from endpoint
float measureVoltage(int nreadings);

#endif // MEASUREMENT_H
