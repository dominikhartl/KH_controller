#include "tmc_driver.h"
#include <TMCStepper.h>
#include "pins.h"
#include "config.h"
#include "config_store.h"

static HardwareSerial tmcSerial(1);
static TMC2209Stepper* sampleDriver = nullptr;
static TMC2209Stepper* titrateDriver = nullptr;
static bool tmcDetected = false;
static bool stallFlag = false;

// Convert SG_RESULT threshold to SGTHRS register value.
// TMC2209 triggers stall when SG_RESULT < 2 * SGTHRS, so SGTHRS = threshold / 2.
static uint8_t sgThresholdToRegister(int stallSG) {
  int val = stallSG / 2;
  if (val > 255) val = 255;
  return (uint8_t)val;
}

static void configureDriver(TMC2209Stepper* driver, int rmsCurrent) {
  driver->rms_current(rmsCurrent);
  driver->microsteps(TMC_MICROSTEPS);
  driver->intpol(true);           // Interpolate to 256 microsteps for smoother, quieter operation
  driver->en_spreadCycle(false);  // StealthChop (quiet); stall via UART SG_RESULT
  driver->SGTHRS(0);             // Disabled at init; enabled per-motor after config applied
  driver->TCOOLTHRS(0xFFFFF);    // Enable StallGuard at all velocities
}

bool initTMCDrivers() {
  tmcSerial.begin(TMC_SERIAL_BAUD, SERIAL_8N1, TMC_UART_RX, TMC_UART_TX);

  sampleDriver = new TMC2209Stepper(&tmcSerial, TMC_R_SENSE, TMC_SAMPLE_ADDR);
  titrateDriver = new TMC2209Stepper(&tmcSerial, TMC_R_SENSE, TMC_TITRATE_ADDR);

  sampleDriver->begin();
  titrateDriver->begin();

  uint8_t sampleVer = sampleDriver->version();
  uint8_t titrateVer = titrateDriver->version();

  if (sampleVer == 0x21 && titrateVer == 0x21) {
    configureDriver(sampleDriver, TMC_SAMPLE_RMS_MA);
    configureDriver(titrateDriver, TMC_TITRATE_RMS_MA);
    // Apply per-motor chopper mode from config store
    // Note: configStore may not be initialized yet (initTMCDrivers runs before configStore.begin).
    // SpreadCycle defaults to false (StealthChop) which is correct for uninitialized NVS.
    // SGTHRS stays at 0 from configureDriver(); applyStallGuardConfig() sets it after configStore.begin().
    sampleDriver->en_spreadCycle(configStore.getSampleSpreadCycle());
    titrateDriver->en_spreadCycle(configStore.getTitrateSpreadCycle());
    pinMode(DIAG_SAMPLE, INPUT);
    pinMode(DIAG_TITRATE, INPUT);
    tmcDetected = true;
    Serial.println("TMC2209 drivers detected and configured");
  } else {
    Serial.printf("TMC2209 not detected (sample=0x%02X, titrate=0x%02X)\n",
                  sampleVer, titrateVer);
    delete sampleDriver;
    delete titrateDriver;
    sampleDriver = nullptr;
    titrateDriver = nullptr;
    tmcDetected = false;
  }

  return tmcDetected;
}

bool isTMCDetected() { return tmcDetected; }

bool isSampleStalled() {
  return tmcDetected && digitalRead(DIAG_SAMPLE);
}

bool isTitrateStalled() {
  return tmcDetected && digitalRead(DIAG_TITRATE);
}

bool wasMotorStall() { return stallFlag; }
void setStallFlag() { stallFlag = true; }
void clearStallFlag() { stallFlag = false; }

uint16_t getSampleSG() {
  return sampleDriver ? sampleDriver->SG_RESULT() : 0;
}

uint16_t getTitrateSG() {
  return titrateDriver ? titrateDriver->SG_RESULT() : 0;
}

void resetSampleStallGuard() {
  if (!sampleDriver) return;
  sampleDriver->SGTHRS(0);        // disable StallGuard → DIAG goes LOW
  delay(10);
  uint8_t reg = sgThresholdToRegister(configStore.getSampleStallSG());
  sampleDriver->SGTHRS(reg);
  sampleDriver->SG_RESULT();      // read to clear any pending state
}

void resetTitrateStallGuard() {
  if (!titrateDriver) return;
  titrateDriver->SGTHRS(0);
  delay(10);
  uint8_t reg = sgThresholdToRegister(configStore.getTitrateStallSG());
  titrateDriver->SGTHRS(reg);
  titrateDriver->SG_RESULT();
}

void clearSampleDIAG() {
  if (!sampleDriver) return;
  sampleDriver->SGTHRS(0);
  delay(1);
  uint8_t reg = sgThresholdToRegister(configStore.getSampleStallSG());
  sampleDriver->SGTHRS(reg);
}

void clearTitrateDIAG() {
  if (!titrateDriver) return;
  titrateDriver->SGTHRS(0);
  delay(1);
  uint8_t reg = sgThresholdToRegister(configStore.getTitrateStallSG());
  titrateDriver->SGTHRS(reg);
}

void disableSampleStallGuard() {
  if (sampleDriver) sampleDriver->SGTHRS(0);
}

void enableSampleStallGuard() {
  if (!sampleDriver) return;
  uint8_t reg = sgThresholdToRegister(configStore.getSampleStallSG());
  sampleDriver->SGTHRS(reg);
}

void disableTitrateStallGuard() {
  if (titrateDriver) titrateDriver->SGTHRS(0);
}

void enableTitrateStallGuard() {
  if (!titrateDriver) return;
  uint8_t reg = sgThresholdToRegister(configStore.getTitrateStallSG());
  titrateDriver->SGTHRS(reg);
}

void applyStallGuardConfig() {
  if (!tmcDetected) return;
  enableSampleStallGuard();
  enableTitrateStallGuard();
}

void setSampleSGTHRS(uint8_t val) {
  if (sampleDriver) sampleDriver->SGTHRS(val);
}

void setTitrateSGTHRS(uint8_t val) {
  if (titrateDriver) titrateDriver->SGTHRS(val);
}

void setSampleSpreadCycle(bool enable) {
  if (sampleDriver) sampleDriver->en_spreadCycle(enable);
}

void setTitrateSpreadCycle(bool enable) {
  if (titrateDriver) titrateDriver->en_spreadCycle(enable);
}

uint32_t getSampleDrvStatus() {
  return sampleDriver ? sampleDriver->DRV_STATUS() : 0;
}

uint32_t getTitrateDrvStatus() {
  return titrateDriver ? titrateDriver->DRV_STATUS() : 0;
}

uint32_t getSampleIOIN() {
  return sampleDriver ? sampleDriver->IOIN() : 0;
}

uint32_t getTitrateIOIN() {
  return titrateDriver ? titrateDriver->IOIN() : 0;
}

void printTMCDebug() {
  if (!tmcDetected) { Serial.println("TMC: not detected"); return; }
  uint32_t gconf_s = sampleDriver->GCONF();
  uint32_t gconf_t = titrateDriver->GCONF();
  Serial.printf("TMC sample:  GCONF=0x%04X SG=%d DIAG=%d TSTEP=%lu\n",
    gconf_s, sampleDriver->SG_RESULT(), digitalRead(DIAG_SAMPLE), sampleDriver->TSTEP());
  Serial.printf("TMC titrate: GCONF=0x%04X SG=%d DIAG=%d TSTEP=%lu\n",
    gconf_t, titrateDriver->SG_RESULT(), digitalRead(DIAG_TITRATE), titrateDriver->TSTEP());
}
