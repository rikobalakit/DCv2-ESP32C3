//
// Created by rico on 11/17/2022.
//

#ifndef DCV2_STEAMDECK_PROTOTYPE_MAIN_H
#define DCV2_STEAMDECK_PROTOTYPE_MAIN_H
#define ARDUINO_ESP32C3_DEV

#endif //DCV2_STEAMDECK_PROTOTYPE_MAIN_H

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_BNO08x.h>
#include <NimBLEDevice.h>
#include <Adafruit_H3LIS331.h>
#include <ESP32_New_ISR_Servo.h>

#define SERVICE_UUID "7ac5d0b9-a214-4c2b-b02a-7d300d756709"
#define ORIENTATION_ALL_UUID "64dc361e-9e25-4ab9-aa07-4813b15f2c83"
#define CTRL_ALL_UUID "1d340766-ffa2-4aed-b03d-cf3796a46d82"

std::string BLEName = "DCv2_SEEED";

bool _displayEnabled = true;
bool _debugEnabled = true;
Adafruit_SSD1306 Display(-1); //-1 arg means no reset pin
Adafruit_BNO08x bno = Adafruit_BNO08x(55);
Adafruit_H3LIS331 lis = Adafruit_H3LIS331();

String _line0, _line1, _line2, _line3, _line4, _line5;

static NimBLEServer *pServer;

float orientationX;
float orientationY;
float orientationZ;
NimBLEAttValue ctrlAllMessage;

#define PIN_VOLTAGE_READER A0
// these pins are good: D1, D2, D3, D8, D9, D10
// these pin work but will already be reserved:
// untested since totally reserved: D4 D5
// these pins will prob be reserved: D6 (uart), D7 (uart), D0/A1 (voltage reading)
#define PIN_TEST_SERVO_L D8
#define PIN_TEST_SERVO_R D9
#define PIN_TEST_SERVO_W0 D10
#define PIN_TEST_SERVO_W1 D3

short safetyOffset = 0;

short ctrlValue0 = 90;
short ctrlValue1 = 90;
short ctrlValue2 = 90;
short ctrlValue3 = 90;

short voltageReadingRaw;
short voltageReadingMv;

int servoLIndex = -1;
int servoRIndex = -1;
int servoW0Index = -1;
int servoW1Index = -1;

bool _interpretString = false;

// copied from french blheli stuff https://www.rcgroups.com/forums/showthread.php?2555162-KISS-ESC-24A-Race-Edition-Flyduino-32bit-ESC

static int16_t ESC_telemetrie[5]; // Temperature, Voltage, Current, used mAh, eRpM

static int16_t W0_Temperature;
static int16_t W0_Voltage;
static int16_t W0_Current;
static int16_t W0_UsedMah;
static int16_t W0_Rpm;


static uint16_t requestTelemetrie = 0;
static uint16_t regularThrottleSignal = 1000;
static uint8_t SerialBuf[10];
static uint8_t receivedBytes = 0;

bool isReadingTelemetry = false;
bool motorOutputsSetDuringTelemetryReading = false;

// setup

void SetupDisplay();

void SetupIMU();

void SetupAccelerometer();

void SetupBLE();

void SetupMotors();

void SetupWeaponTelemetryConnection();

void SetupVoltageReader();

// inputs

void GetIMUData();

void GetAccelerometerData();

void GetVoltageData();

void GetWeaponTelemetry();

uint8_t get_crc8(uint8_t *Buf, uint8_t BufLen);
uint8_t update_crc8(uint8_t crc, uint8_t crc_seed);
void receiveTelemtrie();
// outputs

void SetMotorOutputs();

void SetDataForBroadcast();

// extensions

bool isNumber(const std::string &s);

void DisplayText(float timeSeconds, String line0 = "", String line1 = "", String line2 = "", String line3 = "",
                 String line4 = "", String line5 = "");