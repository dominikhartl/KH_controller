#include "tmc_driver.h"
#include <TMCStepper.h>
#include "pins.h"
#include "config.h"
#include "config_store.h"

static HardwareSerial tmcSerial(1);
static TMC2209Stepper* sampleDriver = nullptr;
static TMC2209Stepper* titrateDriver = nullptr;
static bool tmcDetected = false;

static void configureDriver(TMC2209Stepper* driver, int rmsCurrent) {
  driver->rms_current(rmsCurrent);
  driver->microsteps(TMC_MICROSTEPS);
  driver->intpol(true);           // Interpolate to 256 microsteps for smoother, quieter operation
  driver->en_spreadCycle(false);  // StealthChop (quiet)
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
    // Pin titrate hold current at ~440 mA: run current is raised for burst
    // torque, but standstill heating (the heat-soak driver) must not rise
    titrateDriver->rms_current(TMC_TITRATE_RMS_MA, TMC_TITRATE_HOLD_MULT);
    sampleDriver->en_spreadCycle(configStore.getSampleSpreadCycle());
    titrateDriver->en_spreadCycle(configStore.getTitrateSpreadCycle());
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

