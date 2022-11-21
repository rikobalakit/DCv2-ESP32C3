//
// Created by rico on 11/17/2022.
//

#ifndef DCV2_STEAMDECK_PROTOTYPE_MAIN_H
#define DCV2_STEAMDECK_PROTOTYPE_MAIN_H

#endif //DCV2_STEAMDECK_PROTOTYPE_MAIN_H

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_BNO055.h>
#include <NimBLEDevice.h>
#include <Adafruit_H3LIS331.h>
#include <ESP32Servo.h>

#define SERVICE_UUID "7ac5d0b9-a214-4c2b-b02a-7d300d756709"
#define ORIENTATION_ALL_UUID "64dc361e-9e25-4ab9-aa07-4813b15f2c83"
#define CTRL_ALL_UUID "1d340766-ffa2-4aed-b03d-cf3796a46d82"

std::string BLEName = "DCv2_0";

bool _displayEnabled = true;
bool _debugEnabled = false;
Adafruit_SSD1306 Display(-1); //-1 arg means no reset pin
Adafruit_BNO055 bno = Adafruit_BNO055(55);
Adafruit_H3LIS331 lis = Adafruit_H3LIS331();

String _line0, _line1, _line2, _line3, _line4, _line5;

static NimBLEServer *pServer;

float orientationX;
float orientationY;
float orientationZ;
NimBLEAttValue ctrlAllMessage;

#define PIN_VOLTAGE_READER 0
#define PIN_TEST_SERVO_L 2
#define PIN_TEST_SERVO_R 21
Servo _testServoL;
Servo _testServoR;

short ctrlValue0 = 90;
short ctrlValue1 = 90;

short voltageReadingRaw;
short voltageReadingMv;

bool _interpretString = false;

// setup

void SetupDisplay();

void SetupIMU();

void SetupAccelerometer();

void SetupBLE();

void SetupMotors();

void SetupVoltageReader();

// inputs

void GetIMUData();

void GetAccelerometerData();

void GetVoltageData();

// outputs

void SetMotorOutputs();

void SetDataForBroadcast();

// extensions

bool isNumber(const std::string &s);

void DisplayText(float timeSeconds, String line0 = "", String line1 = "", String line2 = "", String line3 = "",
                 String line4 = "", String line5 = "");