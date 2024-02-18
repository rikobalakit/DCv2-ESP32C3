//
// Created by rico on 11/17/2022.
//

#ifndef DCV2_STEAMDECK_PROTOTYPE_MAIN_H
#define DCV2_STEAMDECK_PROTOTYPE_MAIN_H
#define ARDUINO_ESP32C3_DEV

#endif //DCV2_STEAMDECK_PROTOTYPE_MAIN_H

#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <NimBLEDevice.h>
#include <Adafruit_H3LIS331.h>
#include <Servo.h>
#include <quaternion.h>
#include <DShotESC.h>

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
#define TELEMETRY_ENABLED false

#define DEBUG_

#if (USING_LEGACY_BNO055 != true)
#include <Adafruit_BNO08x.h>
#else
#include <Adafruit_BNO055.h>
#endif

#define DETHROTTLE_WEAPON_FACTOR 4 //last 10 // higher = harder throttle. lower = lighter throttle. 0 = none.
#define DETHROTTLE_WEAPON_HARD_MINIMUM 0.4 // last 0.5 // minimum multiplier
#define DETHROTTLE_WEAPON_ON_HARD_TURN true
#define DETHROTTLE_ANGLE_MINIMUM 10
#define GYRO_LIFT_ANGLE_TOLERANCE 5

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

std::string BLEName = "DataCollector_";
std::string BLENameSuffix = "_";

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

bool flipImu = true;

Adafruit_H3LIS331 lis = Adafruit_H3LIS331();

String _line0, _line1, _line2, _line3, _line4, _line5;

static NimBLEServer *pServer;

vector<uint8_t> _telemetryVector;

float _orientationR; // aka real, or W
float _orientationI; // aka x
float _orientationJ; // aka y
float _orientationK; // aka z

float _orientationYaw;
float _orientationPitch;
float _orientationRoll;

bool _isTilted = false;

float _bnoAccelerationX;
float _bnoAccelerationY;
float _bnoAccelerationZ;

float _lisAccelerationX;
float _lisAccelerationY;
float _lisAccelerationZ;

int8_t _bnoTemp = 0;

uint8_t _bnoCalibrationSystem = 0;
uint8_t _bnoCalibrationGyro = 0;
uint8_t _bnoCalibrationAccelerometer = 0;
uint8_t _bnoCalibrationMagnetometer = 0;


int8_t _settingAngleTolerance;
int _settingAngleToleranceMin = 0;
int _settingAngleToleranceMax = 20;

int8_t _settingTurningMultiplier;
float _settingTurningMultiplierMin = 0.1;
float _settingTurningMultiplierMax = 2.0;

int8_t _settingAdditiveThrottleMultiplier;
float _settingAdditiveThrottleMultiplierMin = 0.1;
float _settingAdditiveThrottleMultiplierMax = 4.0;

int8_t _settingAttenuationWeaponThrottle;
float  _settingAttenuationWeaponThrottleMin = 0;
float  _settingAttenuationWeaponThrottleMax = 20;


int8_t _settingAttenuationWeaponThrottleTurning;
float  _settingAttenuationWeaponThrottleTurningMin = 0;
float  _settingAttenuationWeaponThrottleTurningMax = 0.5;


int _angleTolerance = 3;
float _turningMultiplier = 0.3;
float _additiveThrottleMultiplier = 0.9;
float _attenuationWeaponThrottle = 0.5f;
float _attenuationWeaponThrottleTurning = 0.5f;

bool _isInverted = false;

NimBLEAttValue _controlMessage;

#define PIN_VOLTAGE_READER A0
// these pins are good: D1, D2, D3, D8, D9, D10
// these pin work but will already be reserved:
// untested since totally reserved: D4 D5
// these pins will prob be reserved: D6 (uart), D7 (uart), D0/A1 (voltage reading)
#define PIN_TEST_SERVO_L D3
#define PIN_TEST_SERVO_R D10
#define PIN_TEST_SERVO_W0 D9
#define PIN_TEST_SERVO_W1 D8
#define PIN_NUM_NEOPIXEL_OUTPUT D2

#define MOTOR_L_USES_DSHOT false
#define MOTOR_R_USES_DSHOT false
#define MOTOR_W0_USES_DSHOT false
#define MOTOR_W1_USES_DSHOT false

// RGBLEDs

#define TOTAL_LED 8
#define MAX_BRIGHTNESS 80
#define MAX_HUE 255
#define HUE_BLUE 171
#define LED_BOARD 0
#define LED_CENTER 1
#define LED_L_DRIVE 6
#define LED_R_DRIVE 4
#define LED_W1 2
#define LED_TILT 5
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

#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))

#define TURN_FORWARD_BIAS 4 //can be negative. unit is servo degrees.
#define MINIMUM_TURNING_THROTTLE 15 // unit is servo degrees

Adafruit_NeoPixel strip(TOTAL_LED, PIN_NUM_NEOPIXEL_OUTPUT, NEO_GRB + NEO_KHZ800);

Servo allServos = Servo();

/*
CRGB _leds[TOTAL_LED];
CRGB _colorWhite = CHSV(0, 0, MAX_BRIGHTNESS);
CRGB _colorGrey = CHSV(0, 0, MAX_BRIGHTNESS / 10);
 */

int _wheelIndex = 2; // this is spinning pinwheel animation frame

bool _bluetoothClientExists = false;
bool _receivedHeartbeatFromClient = true; 

int16_t _throttleLeftDrive = 90;
int16_t _throttleRightDrive = 90;
int16_t _throttleWeapon0 = 90;
int16_t _throttleWeapon1 = 90;

int16_t _smartThrottleDrive = 0;
int16_t _smartHeadingTarget = 0;

int16_t _currentSmartThrottleLeftDrive = 0;
int16_t _currentSmartThrottleRightDrive = 0;
int16_t _currentSmartThrottleWeapon1 = 0;
float _currentSmoothThrottleWeapon1 = 90;
float _rampupTime = 2;

bool _headingResetEngaged = false;
int16_t _smartHeadingOffset = 0;

uint32_t _colorWhite;
uint32_t _colorGreen;
uint32_t _colorYellow;
uint32_t _colorRed;
uint32_t _colorBlack;
uint32_t _colorPurple;
uint32_t _colorBlue;
uint32_t _colorTeal;


bool _buttonAPressed = false;
bool _buttonBPressed = false;
bool _buttonXPressed = false;
bool _buttonYPressed = false;
bool _buttonDUPressed = false;
bool _buttonDDPressed = false;
bool _buttonDLPressed = false;
bool _buttonDRPressed = false;
bool _buttonL1Pressed = false;
bool _buttonR1Pressed = false;
bool _joystickLEngaged = false;
bool _joystickREngaged = false;

#define  HEADING_JOYSTICK_NOT_PRESSED 1000
#define  HEADING_RESET 1001

int16_t _voltageReadingRaw;
int16_t _voltageReadingMillivolts;

int _servoLIndex = -1;
int _servoRIndex = -1;
int _servoW0Index = -1;
int _servoW1Index = -1;

bool _interpretString = false;

#define TIMING_MEASUREMENT_SAMPLES 16
#define MINIMUM_VOLTAGE_BATTERY_FULL 15200
#define MINIMUM_VOLTAGE_BATTERY_HIGH 15200
#define MINIMUM_VOLTAGE_BATTERY_LOW 14000
#define MINIMUM_VOLTAGE_BATTERY_DEAD 13200
#define MINIMUM_VOLTAGE_USING_USB 6000
long _timingMeasurementBuffer[TIMING_MEASUREMENT_SAMPLES];
int _currentTimingMeasurementBufferIndex = 0;
long _lastCycleTime = 0;
long _timeSinceLastPwmUpdateUs = 0;

// copied from french blheli stuff https://www.rcgroups.com/forums/showthread.php?2555162-KISS-ESC-24A-Race-Edition-Flyduino-32bit-ESC

static int16_t _escTelemetryValues[5]; // Temperature, Voltage, Current, used mAh, eRpM

static int16_t _w0_Temperature = 0x00;
static int16_t _w0_Voltage = 0x00;
static int16_t _w0_Current = 0x00;
static int16_t _w0_UsedMah = 0x00;
static int16_t _w0_Rpm = 0x00;

#define DELAY_BEFORE_TELEMETRY_COMES_BACK 20000
#define TELEMETRY_READ_TIMEOUT 50000
#define DELAY_BEFORE_STARTING_TELEMETRY 5000

static uint8_t _serialBuffer[10];
static byte _bnoCalibrationOffsets[22];
static uint8_t _receivedBytes = 0;
long _timeTelemetrySignalSentMicros;
int _totalDiscardedBytes = 0;


short _securityByteValidation = 0x69;
bool _passesSecurityByteValidation = false;
bool _lastHeartbeatValid = false;
long _previousHeartbeatTimeMillis;
long _currentHeartbeatTimeMillis;
long _timeLastReceivedHeartbeatMillis;

long _millisSinceLastMotorLoop = 0;

long _lastTimeLedsUpdated = 0;
long _ledUpdateCooldownTime = 50;

long _lastLoopStartTime = 0;

#define TIMEOUT_HEARTBEAT_LOST 2000
#define TIMEOUT_HEARTBEAT_LOST_REBOOT 10000
int _skippedHeartbeats = 0;

//dshot stuff
const auto FAILSAFE_THROTTLE = 999;
const auto INITIAL_THROTTLE = 1048;

#if MOTOR_L_USES_DSHOT
DShotESC motorLeftDrive;
#endif

#if MOTOR_L_USES_DSHOT
DShotESC motorRightDrive;
#endif

#if MOTOR_L_USES_DSHOT
DShotESC motorWeapon0;
#endif

#if MOTOR_L_USES_DSHOT
DShotESC motorWeapon1;
#endif

// setup

uint16_t DegreesToDShotThrottle(int degreesInput);

void InitializeDisplay();

void InitializeImu();

void InitializeAccelerometer();

void InitializeBluetooth();

void EnableMotors();

void InitializeEscTelemetry();

void InitializeVoltageReader();

void InitializeLeds();

void printMac(const unsigned char *mac);

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
int WrapEulerAngle(int inputAngle);
int GetAngleDeltaDegrees(int angle0, int angle1);
int GetOffsetHeading();

void SetMotorOutputs();

void GetAndSetBluetoothData();

void SetWeaponTelemetrySignal();

void SetFailsafe();

void SetLeds();

void SetMainLeds(uint32_t color);

void SetTimingData();

bool GetFlashValue(int periodMilliseconds, bool startsTrue = true);

void RestartOnButtonCombination();

void SetFormulaId();

// extensions

void SafeSerialPrintLn(String lineToPrint);

bool isNumber(const std::string &s);

void DisplayText(float timeSeconds, String line0 = "", String line1 = "", String line2 = "", String line3 = "",
                 String line4 = "", String line5 = "");