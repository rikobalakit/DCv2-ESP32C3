//
// Created by rico on 11/17/2022.
//

#ifndef DCV2_STEAMDECK_PROTOTYPE_MAIN_H
#define DCV2_STEAMDECK_PROTOTYPE_MAIN_H
#define ARDUINO_ESP32C3_DEV

#endif //DCV2_STEAMDECK_PROTOTYPE_MAIN_H

#include <Arduino.h>
#include <SPI.h>
#include <FastLED.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include <NimBLEDevice.h>
#include <Adafruit_H3LIS331.h>
#include <ESP32_New_ISR_Servo.h>

#define SERVICE_UUID "7ac5d0b9-a214-4c2b-b02a-7d300d756709"
#define TELEMETRY_ALL_UUID "64dc361e-9e25-4ab9-aa07-4813b15f2c83"
#define CTRL_ALL_UUID "1d340766-ffa2-4aed-b03d-cf3796a46d82"

// Want to declare a list of safe client addresses
// steam deck
// ipad
// iphone
// laptop

#define LEDS_ENABLED true
#define DISPLAY_ENABLED false
#define BLUETOOTH_ENABLED true
#define IMU_ENABLED true
#define USING_LEGACY_BNO055 false
#define ACCELEROMETER_ENABLED true
#define VOLTAGE_READER_ENABLED true
#define MOTORS_ENABLED true
#define TELEMETRY_ENABLED true

#define DEBUG_

#if (USING_LEGACY_BNO055 != true)
#include <Adafruit_BNO08x.h>
#else
#include <Adafruit_BNO055.h>
#endif

using namespace std;

/*
 * TIMING STUFF:
 * Absolutely nothing: 0,150 us
 * LEDs: 2,350 us
 * Display: 94,000 us
 * Bluetooth: 0,075us no clients, 0,358us with client connected (incl talking)
 * IMU: unknown
 * Accelerometer: 1,564us with reads
 * Voltage Reader: 0,065uS
 * Motors: 310us
 * Motors + Telemetry: 40,000uS (25Hz!!!)
 */

std::string BLEName = "DCv2_SEEED_A";

bool _displayEnabledAndFound = false;
bool _imuEnabledAndFound = false;
bool _accelerometerEnabledAndFound = false;
bool _serialDebugEnabled = true;
bool _debugEnabled = false;

bool _pwmTimingDebugEnabled = false;
bool _roundTripTimingDebugEnabled = false;

Adafruit_SSD1306 Display(-1); //-1 arg means no reset pin


#if (USING_LEGACY_BNO055 != true)
Adafruit_BNO08x bno = Adafruit_BNO08x();
#else
Adafruit_BNO055 bno = Adafruit_BNO055(55);
#endif

sensors_event_t event; //I dont think this can be renamed either

Adafruit_H3LIS331 lis = Adafruit_H3LIS331();

String _line0, _line1, _line2, _line3, _line4, _line5;

static NimBLEServer *pServer;

vector<uint8_t> TelemetryVector;

float orientationR; // aka real, or W
float orientationI; // aka x
float orientationJ; // aka y
float orientationK; // aka z

float BNOAccelerationX;
float BNOAccelerationY;
float BNOAccelerationZ;

float LISAccelerationX;
float LISAccelerationY;
float LISAccelerationZ;

int8_t BNOTemp = 0;

uint8_t BNOCalibrationSystem = 0;
uint8_t BNOCalibrationGyro = 0;
uint8_t BNOCalibrationAccelerometer = 0;
uint8_t BNOCalibrationMagnetometer = 0;

NimBLEAttValue ctrlAllMessage;

#define PIN_VOLTAGE_READER A0
// these pins are good: D1, D2, D3, D8, D9, D10
// these pin work but will already be reserved:
// untested since totally reserved: D4 D5
// these pins will prob be reserved: D6 (uart), D7 (uart), D0/A1 (voltage reading)
#define PIN_TEST_SERVO_L D8
#define PIN_TEST_SERVO_R D3
#define PIN_TEST_SERVO_W0 D9
#define PIN_TEST_SERVO_W1 D10
#define PIN_NUM_NEOPIXEL_OUTPUT D2

// RGBLEDs

#define TOTAL_LED 8
#define MAX_BRIGHTNESS 80
#define HUE_BLUE 171
#define LED_BOARD 0
#define LED_CENTER 1
// These correspond to the positions on a clock
#define LED_12 2
#define LED_2 3
#define LED_4 4
#define LED_6 5
#define LED_8 6
#define LED_10 7

#define SAT_FORWARD 0
#define HUE_FORWARD 96

#define HUE_NEUTRAL 213
#define SAT_NEUTRAL 255

#define HUE_REVERSE 0
#define SAT_REVERSE 255

#define HUE_DISCONNECTED 45

#define HUE_PURPLE 213
#define STARTING_BRIGHTNESS 40
CRGB leds[TOTAL_LED];
CRGB DC_White = CHSV(0, 0, MAX_BRIGHTNESS);
CRGB DC_Grey = CHSV(0, 0, MAX_BRIGHTNESS/10);

int wheelIndex = 2;

bool bluetoothClientExists = false;
bool receivedHeartbeat = true; //faked for now

int16_t safetyOffset = 0;

int16_t throttleLeftDrive = 90;
int16_t throttleRightDrive = 90;
int16_t throttleWeapon0 = 90;
int16_t throttleWeapon1 = 90;

int16_t voltageReadingRaw;
int16_t voltageReadingMv;

int servoLIndex = -1;
int servoRIndex = -1;
int servoW0Index = -1;
int servoW1Index = -1;

bool _interpretString = false;

#define TIMING_MEASUREMENT_SAMPLES 16
#define MINIMUM_VOLTAGE_BATTERY_FULL 15600
#define MINIMUM_VOLTAGE_BATTERY_LOW 14000
#define MINIMUM_VOLTAGE_BATTERY_DEAD 13200
#define MINIMUM_VOLTAGE_USING_USB 6000
long timingMeasurementBuffer[TIMING_MEASUREMENT_SAMPLES];
int currentTimingMeasurementBufferIndex = 0;
long lastCycleTime = 0;

// copied from french blheli stuff https://www.rcgroups.com/forums/showthread.php?2555162-KISS-ESC-24A-Race-Edition-Flyduino-32bit-ESC

static int16_t ESC_telemetrie[5]; // Temperature, Voltage, Current, used mAh, eRpM

static int16_t W0_Temperature = 0x00;
static int16_t W0_Voltage = 0x00;
static int16_t W0_Current = 0x00;
static int16_t W0_UsedMah = 0x00;
static int16_t W0_Rpm = 0x00;

#define DELAY_BEFORE_TELEMETRY_COMES_BACK 20000
#define TELEMETRY_READ_TIMEOUT 50000
#define DELAY_BEFORE_STARTING_TELEMETRY 5000
static uint16_t requestTelemetrie = 0;
static uint16_t regularThrottleSignal = 1000;
static uint8_t SerialBuf[10];
static byte bnoCalibrationOffsets[22];
static uint8_t receivedBytes = 0;
long timeTelemetrySignalSentMicros;

int totalDiscardedBytes = 0;
// setup

void InitializeDisplay();

void InitializeImu();

void InitializeAccelerometer();

void InitializeBluetooth();

void InitializeMotors();

void InitializeEscTelemetry();

void InitializeVoltageReader();

void InitializeLeds();

// inputs

void GetIMUData();

void GetAccelerometerData();

void GetVoltageData();

void GetWeaponTelemetry();

long GetAverageCyclePeriodMilliseconds();

uint8_t get_crc8(uint8_t *Buf, uint8_t BufLen);
uint8_t update_crc8(uint8_t crc, uint8_t crc_seed);
void receiveTelemtrie();
// outputs

void SetMotorOutputs();

void GetAndSetBluetoothData();

void SetWeaponTelemetrySignal();

void SetLeds();

void SetMainLeds(CRGB color);

void SetTimingData();

bool GetFlashValue(int periodMilliseconds, bool startsTrue = true);

// extensions

void SafeSerialPrintLn(String lineToPrint);

bool isNumber(const std::string &s);

void DisplayText(float timeSeconds, String line0 = "", String line1 = "", String line2 = "", String line3 = "",
                 String line4 = "", String line5 = "");