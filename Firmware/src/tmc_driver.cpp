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

static void configureDriver(TMC2209Stepper* driver, int rmsCurrent) {
  driver->rms_current(rmsCurrent);
  driver->microsteps(TMC_MICROSTEPS);
  driver->en_spreadCycle(false);  // StealthChop (quiet); stall via UART SG_RESULT
  driver->SGTHRS(TMC_STALL_THRESHOLD);
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

void setSampleSpreadCycle(bool enable) {
  if (sampleDriver) sampleDriver->en_spreadCycle(enable);
}

void setTitrateSpreadCycle(bool enable) {
  if (titrateDriver) titrateDriver->en_spreadCycle(enable);
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
