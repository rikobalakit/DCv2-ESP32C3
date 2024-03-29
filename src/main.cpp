﻿#include <main.h>

// lifecycle

void setup()
{
    Serial.begin(115200);
    Serial.setTimeout(1);
    SafeSerialPrintLn("Beginning serial");
    SetFormulaId();
    delay(100);
    Wire.begin();
    delay(100);
#if LEDS_ENABLED
    InitializeLeds();
#endif

#if DISPLAY_ENABLED
    InitializeDisplay();
#endif

#if BLUETOOTH_ENABLED
    InitializeBluetooth();
#endif

#if IMU_ENABLED
    InitializeImu();
#endif

#if ACCELEROMETER_ENABLED
    InitializeAccelerometer();
#endif

#if VOLTAGE_READER_ENABLED
    InitializeVoltageReader();
#endif

#if MOTORS_ENABLED
    //EnableMotors();
#endif

#if TELEMETRY_ENABLED
    InitializeEscTelemetry();
#endif

#if LEDS_ENABLED
    strip.fill(_colorBlack, 0, PIN_NUM_NEOPIXEL_OUTPUT);
#endif
}

void loop()
{

    while (millis() - _lastLoopStartTime < 10)
    {
        delay(4);
    }

    _lastLoopStartTime = millis();
    delayMicroseconds(10); // this is just in for safety...

    SetFailsafe();


    SafeSerialPrintLn(String(BLENameSuffix.c_str()));
    unsigned char thisEspMacAddress[6] = {0};
    esp_efuse_mac_get_default(thisEspMacAddress);
    printMac(thisEspMacAddress);


    if (_pwmTimingDebugEnabled)
    {
        while (true)
        {
            SetMotorOutputs();

            _throttleLeftDrive = 0;
            _throttleRightDrive = 0;
            _throttleWeapon0 = 0;
            _throttleWeapon1 = 0;

            delay(1000);

            SetMotorOutputs();

            _throttleLeftDrive = 180;
            _throttleRightDrive = 180;
            _throttleWeapon0 = 180;
            _throttleWeapon1 = 180;

            delay(1000);
        }
    }

#if TELEMETRY_ENABLED
    // must be done at the start to give the most time for a response signal to come back
    SetWeaponTelemetrySignal();
#endif

    while ((micros() - _timeTelemetrySignalSentMicros) < DELAY_BEFORE_TELEMETRY_COMES_BACK)
    {}
    {
        SetTimingData();

#if MOTORS_ENABLED
        SetMotorOutputs();
#endif

#if IMU_ENABLED
        GetIMUData();
#endif

#if ACCELEROMETER_ENABLED
        GetAccelerometerData();
#endif

#if VOLTAGE_READER_ENABLED
        GetVoltageData();
#endif

#if BLUETOOTH_ENABLED
        GetAndSetBluetoothData();
#endif

#if DISPLAY_ENABLED
        DisplayText(0.01, _line0, _line1, _line2, _line3, _line4, _line5);
#endif

#if LEDS_ENABLED
        SetLeds();
#endif
    }

#if TELEMETRY_ENABLED
    GetWeaponTelemetry();
#endif

    RestartOnButtonCombination();
    //SafeSerialPrintLn(" BNOX " + String(_bnoAccelerationX) + " LISX " + String(_lisAccelerationX) +
    //                  ", BNOY " + String(_bnoAccelerationY) + " LISY " + String(_lisAccelerationY) +
    //                  ", BNOZ " + String(_bnoAccelerationZ) + " LISZ " + String(_lisAccelerationZ));
}

// setup

void InitializeDisplay()
{
    if (!Display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
    {
        _displayEnabledAndFound = false;
    }
    else
    {
        DisplayText(1, "DCv2 Prototype");
        _displayEnabledAndFound = true;

        _line0 = "";
        _line1 = "";
        _line2 = "";
        _line3 = "";
        _line4 = "";
        _line5 = "";
    }
}

void InitializeImu()
{
    delay(100);

#if (USING_LEGACY_BNO055 != true)
    // This means initialize BNO085 instead
    if (!bno.begin_I2C())
    {
        DisplayText(1, "IMU Error", "Bad Init");
        _imuEnabledAndFound = false;
        SafeSerialPrintLn("IMU Error, BN0085");

        for (int i = 0; i < 5; i++)
        {
            SetMainLeds(_colorYellow);
            strip.show();
            delay(50);
            SetMainLeds(_colorBlack);
            strip.show();
            delay(50);
        }

        //esp_restart();
    }
    else
    {
        DisplayText(0.25, "IMU Success");
        bno.enableReport(SH2_GAME_ROTATION_VECTOR, 25000);
        bno.enableReport(SH2_ACCELEROMETER, 25000);
        //bno.enableReport(SH2_TEMPERATURE);

        _imuEnabledAndFound = true;
        SafeSerialPrintLn("IMU success, BN0085");
    }
#else
    if (!bno.begin(OPERATION_MODE_NDOF))
    {
        /* There was a problem detecting the BNO055 ... check your connections */
        Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        _imuEnabledAndFound = false;
    }
    
    else
    {
        _imuEnabledAndFound = true;
    }
    
    delay(10);

    if (_imuEnabledAndFound)
    {
        bno.setExtCrystalUse(true);

    }
#endif
}

void InitializeAccelerometer()
{
    if (!lis.begin_I2C())
    {
        DisplayText(1, "Accel Error", "Bad Init");
        _accelerometerEnabledAndFound = false;
        SafeSerialPrintLn("Accel Error, H3LIS331");

        for (int i = 0; i < 5; i++)
        {
            SetMainLeds(_colorBlue);
            strip.show();
            delay(50);
            SetMainLeds(_colorBlack);
            strip.show();
            delay(50);
        }

        //esp_restart();
    }
    else
    {
        DisplayText(0.25, "Accel Success");
        lis.setRange(H3LIS331_RANGE_400_G);
        _accelerometerEnabledAndFound = true;
        SafeSerialPrintLn("Accel Success");
    }
}

void InitializeLeds()
{
    pinMode(PIN_NUM_NEOPIXEL_OUTPUT, OUTPUT);
    delay(1);
    strip.begin();
    strip.show();


    _colorWhite = strip.Color(MAX_BRIGHTNESS, MAX_BRIGHTNESS, MAX_BRIGHTNESS);
    _colorBlack = strip.Color(0, 0, 0);

    _colorRed = strip.Color(MAX_BRIGHTNESS, 0, 0);
    _colorYellow = strip.Color(MAX_BRIGHTNESS, MAX_BRIGHTNESS, 0);
    _colorGreen = strip.Color(0, MAX_BRIGHTNESS, 0);
    _colorTeal = strip.Color(0, MAX_BRIGHTNESS, MAX_BRIGHTNESS);
    _colorBlue = strip.Color(0, 0, MAX_BRIGHTNESS);
    _colorPurple = strip.Color(MAX_BRIGHTNESS, 0, MAX_BRIGHTNESS);


}

bool CompareAddresses(unsigned char address1[], unsigned char address2[])
{
    if (address1[0] == address2[0]
        && address1[1] == address2[1]
        && address1[2] == address2[2]
        && address1[3] == address2[3]
        && address1[4] == address2[4]
        && address1[5] == address2[5])
    {
        return true;
    }

    return false;
}

void printMac(const unsigned char *mac)
{
    printf("%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

void SetFormulaId()
{
    unsigned char thisEspMacAddress[6] = {0};
    esp_efuse_mac_get_default(thisEspMacAddress);




    // during R&D, this is the "free" one
    // note: the last digit is -2 from the bluetooth address visible in network
    //58:CF:79:EA:D9:D8
    unsigned char formulaAMacAddress[] = {0x58, 0xCF, 0x79, 0xEA, 0xD9, 0xD8};
    unsigned char formulaBMacAddress[] = {0x58, 0xCF, 0x79, 0xF0, 0xA2, 0xB4};
    unsigned char formulaCMacAddress[] = {0x58, 0xCF, 0x79, 0xEA, 0xCE, 0xFC};

    if (thisEspMacAddress[0] == formulaAMacAddress[0]
        && thisEspMacAddress[1] == formulaAMacAddress[1]
        && thisEspMacAddress[2] == formulaAMacAddress[2]
        && thisEspMacAddress[3] == formulaAMacAddress[3]
        && thisEspMacAddress[4] == formulaAMacAddress[4]
        && thisEspMacAddress[5] == formulaAMacAddress[5])
    {
        BLENameSuffix = "A";
        return;
    }
    if (thisEspMacAddress[0] == formulaBMacAddress[0]
        && thisEspMacAddress[1] == formulaBMacAddress[1]
        && thisEspMacAddress[2] == formulaBMacAddress[2]
        && thisEspMacAddress[3] == formulaBMacAddress[3]
        && thisEspMacAddress[4] == formulaBMacAddress[4]
        && thisEspMacAddress[5] == formulaBMacAddress[5])
    {
        BLENameSuffix = "B";
        return;
    }
    if (CompareAddresses(thisEspMacAddress, formulaCMacAddress))
    {
        BLENameSuffix = "C";
        return;
    }


    BLENameSuffix = "PEE"; // error
}


void InitializeBluetooth()
{
    NimBLEDevice::init(BLEName + BLENameSuffix);
    pServer = NimBLEDevice::createServer();

    NimBLEService *pService = pServer->createService(SERVICE_UUID);

    uint16_t conn_handle = 0xFFFE;
    uint16_t minInterval = 1;
    uint16_t maxInterval = 10;
    uint16_t latency = 1000;
    uint16_t timeout = 1000;
    pServer->updateConnParams(conn_handle, minInterval, maxInterval, latency, timeout);

    NimBLECharacteristic *pOrientationAll = pService->createCharacteristic(
            TELEMETRY_ALL_UUID,
            NIMBLE_PROPERTY::READ |
            NIMBLE_PROPERTY::NOTIFY
    );

    pOrientationAll->setValue("INIT");

    NimBLECharacteristic *pCtrlAll = pService->createCharacteristic(
            CTRL_ALL_UUID,
            NIMBLE_PROPERTY::READ |
            NIMBLE_PROPERTY::WRITE_NR
    );

    int16_t neutralPosition = (int16_t) 90;

    byte orientationByte[8];
    orientationByte[0] = neutralPosition & 0x00ff;
    orientationByte[1] = (neutralPosition & 0xff00) >> 8;
    orientationByte[2] = neutralPosition & 0x00ff;
    orientationByte[3] = (neutralPosition & 0xff00) >> 8;
    orientationByte[4] = neutralPosition & 0x00ff;
    orientationByte[5] = (neutralPosition & 0xff00) >> 8;
    orientationByte[6] = neutralPosition & 0x00ff;
    orientationByte[7] = (neutralPosition & 0xff00) >> 8;


    pCtrlAll->setValue(orientationByte);

    pService->start();
    // BLEAdvertising *pAdvertising = pServer->getAdvertising();  // this still is working for backward compatibility
    NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
    pAdvertising->setMinPreferred(0x12);
    pAdvertising->start();

    DisplayText(0.25, "BLE Success", String(BLEName.c_str()));
    _line5 = "BLE Name: " + String(BLEName.c_str());
}

void EnableMotors()
{
    DisplayText(0.25, "Attaching motors", "Pins: " + String(PIN_TEST_SERVO_L) + ", " + String(PIN_TEST_SERVO_R));

#if MOTOR_L_USES_DSHOT
    motorLeftDrive.install((gpio_num_t) PIN_TEST_SERVO_L, RMT_CHANNEL_0);
    motorLeftDrive.init();
    motorLeftDrive.setReversed(false);
    motorLeftDrive.set3DMode(true);

    for (int i = 0; i < 5; i++)
    {
        motorLeftDrive.beep(i);
    }
#else
    if(allServos.detached(PIN_TEST_SERVO_L))
    {
        allServos.attach(PIN_TEST_SERVO_L, 1000, 2000);
    }
#endif

#if MOTOR_R_USES_DSHOT
    motorRightDrive.install((gpio_num_t) PIN_TEST_SERVO_R, RMT_CHANNEL_1);
    motorRightDrive.init();
    motorRightDrive.set3DMode(true);

    for (int i = 0; i < 5; i++)
    {
        motorRightDrive.beep(i);
    }

#else
    if(allServos.detached(PIN_TEST_SERVO_R))
    {
        allServos.attach(PIN_TEST_SERVO_R, 1000, 2000);
    }
#endif

#if MOTOR_W0_USES_DSHOT
    motorWeapon0.install((gpio_num_t) PIN_TEST_SERVO_W0, RMT_CHANNEL_2);
    motorWeapon0.init();
    motorWeapon0.set3DMode(true);
#else
    if(allServos.detached(PIN_TEST_SERVO_W0))
    {
        allServos.attach(PIN_TEST_SERVO_W0, 1000, 2000);
    }
#endif

#if MOTOR_W1_USES_DSHOT
    motorWeapon1.install((gpio_num_t) PIN_TEST_SERVO_W1, RMT_CHANNEL_3);
    motorWeapon1.init();
    motorWeapon1.set3DMode(true);
#else
    if(allServos.detached(PIN_TEST_SERVO_W1))
    {
        allServos.attach(PIN_TEST_SERVO_W1, 1000, 2000);
    }
#endif


}

void DisableMotors()
{
#if MOTOR_L_USES_DSHOT
    //Not yet implemented!
#else
    if(!allServos.detached(PIN_TEST_SERVO_L))
    {
        allServos.detach(PIN_TEST_SERVO_L);
    }
#endif

#if MOTOR_R_USES_DSHOT
    //Not yet implemented!
#else
    if(!allServos.detached(PIN_TEST_SERVO_R))
    {
        allServos.detach(PIN_TEST_SERVO_R);
    }
#endif

#if MOTOR_W0_USES_DSHOT
    //Not yet implemented!
#else
    if(!allServos.detached(PIN_TEST_SERVO_W0))
    {
        allServos.detach(PIN_TEST_SERVO_W0);
    }
#endif

#if MOTOR_W1_USES_DSHOT
    //Not yet implemented!
#else
    if(!allServos.detached(PIN_TEST_SERVO_W1))
    {
        allServos.detach(PIN_TEST_SERVO_W1);
    }
#endif

}

void InitializeVoltageReader()
{
    pinMode(PIN_VOLTAGE_READER, INPUT_PULLDOWN);
}

// inputs

int GetOffsetHeading()
{
    return _orientationYaw + _smartHeadingOffset;
}

void GetIMUData()
{
    if (!_imuEnabledAndFound)
    {
        return;
    }
#if USING_LEGACY_BNO055 == false
    int reportsRead = 0;
    int tempReports = 0;
    int accelReports = 0;
    int orientationReports = 0;

    long microsAtStart = micros();

    sh2_SensorValue_t sensorValues;

    while (bno.getSensorEvent(&sensorValues))
    {
        reportsRead++;

        switch (sensorValues.sensorId)
        {
            case SH2_ACCELEROMETER:
                _bnoAccelerationX = -sensorValues.un.accelerometer.x;
                _bnoAccelerationY = sensorValues.un.accelerometer.y;
                _bnoAccelerationZ = sensorValues.un.accelerometer.z;
                _bnoCalibrationAccelerometer = sensorValues.status && 0x00000011;
                accelReports++;

                if (_bnoAccelerationZ < 0)
                {
                    // is right side up
                    _isInverted = false;
                }
                else
                {
                    // is upside down
                    _isInverted = true;
                }

                //SafeSerialPrintLn("value accel z: " + String(_bnoAccelerationZ) + ", is upside down? " + String(_isInverted));

                break;
            case SH2_GAME_ROTATION_VECTOR:
                _orientationR = sensorValues.un.gameRotationVector.real;
                _orientationI = sensorValues.un.gameRotationVector.i;
                _orientationJ = sensorValues.un.gameRotationVector.j;
                _orientationK = sensorValues.un.gameRotationVector.k;

                imu::Quaternion currentOrientation(_orientationR, _orientationI, _orientationJ, _orientationK);

                auto orientationEuler = currentOrientation.toEuler();

                //SafeSerialPrintLn(
                //"euler angles: x:" + String(orientationEuler.x() * 57.2958) + " y: " + String(orientationEuler.y()* 57.2958) +
                //       " z: " + String(orientationEuler.z()* 57.2958));

                _orientationYaw = orientationEuler.x() * -57.2958;

                _orientationPitch = orientationEuler.z() * -57.2958;
                _orientationRoll = orientationEuler.y() * 57.2958;

                if (flipImu)
                {
                    _orientationYaw = orientationEuler.x() * -57.2958 + 180;
                    _orientationPitch = orientationEuler.z() * 57.2958 + 180;
                }

                //SafeSerialPrintLn(
                //        "euler angles: pitch:" + String(_orientationPitch) + " yaw: " + String(_orientationYaw) +
                //        " roll: " + String(_orientationRoll));

                _bnoCalibrationGyro = sensorValues.status && 0x00000011;
                _bnoCalibrationMagnetometer = sensorValues.status && 0x00000011;
                orientationReports++;

                if (_headingResetEngaged)
                {
                    _smartHeadingOffset = -_orientationYaw;
                }

                if (_orientationRoll > 5 || _orientationRoll < -5)
                {
                    _isTilted = true;
                }
                else
                {
                    _isTilted = false;
                }

                break;
                /*
            case SH2_TEMPERATURE:
                _bnoTemp = (int8_t) sensorValues.un.temperature.value;
                tempReports++;
                 */
        }
    }
    long microsDelta = micros() - microsAtStart;

    _bnoCalibrationSystem = 3;
    //SafeSerialPrintLn("Read " + String(reportsRead) + " reports from IMU in " + String(microsDelta) + " us. Temp:" +
    //                  String(tempReports) + ", Accel: " + String(accelReports) + ", Gyro: " +
    //                  String(orientationReports));

#else
    bno.getEvent(&event);

    imu::Quaternion quat = bno.getQuat();

    _orientationR = quat.w();
    _orientationI = quat.x();
    _orientationJ = quat.y();
    _orientationK = quat.z();

    _bnoAccelerationX = event.acceleration.x;
    _bnoAccelerationY = event.acceleration.y;
    _bnoAccelerationZ = event.acceleration.z;

    _bnoTemp = bno.getTemp();

    bno.getCalibration(&_bnoCalibrationSystem, &_bnoCalibrationGyro, &_bnoCalibrationAccelerometer, &_bnoCalibrationMagnetometer);

    
    bno.getSensorOffsets(_bnoCalibrationOffsets);

#endif

}

void GetAccelerometerData()
{
    if (!_accelerometerEnabledAndFound)
    {
        return;
    }

    lis.getEvent(&event);

    _lisAccelerationX = -event.acceleration.x;
    _lisAccelerationY = event.acceleration.y;
    _lisAccelerationZ = event.acceleration.z;
}

void GetVoltageData()
{
    _voltageReadingRaw = analogRead(PIN_VOLTAGE_READER);
    // batt is 15.38, v after divider is 1.392, raw is
    _voltageReadingMillivolts = _voltageReadingRaw * 7.868858 - 275.40983;
}

void InitializeEscTelemetry()
{
    Serial0.begin(115200); // open serial0 for serial monitor
}

void SetWeaponTelemetrySignal()
{
    if (millis() < DELAY_BEFORE_STARTING_TELEMETRY)
    {
        return;
    }

    ulong telemetryRequestWidth = 50;

    //ESP32_ISR_Servos.setPulseWidth(_servoW0Index, telemetryRequestWidth);
    _timeTelemetrySignalSentMicros = micros();

    //delayMicroseconds(REFRESH_INTERVAL);
}

int WrapEulerAngle(int inputAngle)
{
    while (inputAngle > 180)
    {
        inputAngle -= 360;
    }

    while (inputAngle < -180)
    {
        inputAngle += 360;
    }
    return inputAngle;
}

int GetAngleDeltaDegrees(int currentAngle, int targetAngle)
{
    auto delta = (targetAngle - currentAngle + 360) % 360 - 360;


    return WrapEulerAngle(delta);
}

void GetWeaponTelemetry()
{

    _receivedBytes = 0;

    //read in telemetry from serial
    ulong microsAtStart = micros();

    while ((micros() - _timeTelemetrySignalSentMicros < (TELEMETRY_READ_TIMEOUT)) && _receivedBytes < 10)
    {
        if (Serial0.available())
        {
            _serialBuffer[_receivedBytes] = Serial0.read();
            _receivedBytes++;
        }

    }

    ulong microsAfterReceiveSignal = micros();

    if (_receivedBytes >= 9)
    {
        uint8_t crc8 = get_crc8(_serialBuffer, 9); // get the 8 bit CRC

        if (crc8 != _serialBuffer[9])
        {
            //DisplayText(0.1f, "failed read", "corrupt data");
            _line5 = "corrupt read";
        }
        else
        {
            _escTelemetryValues[0] = _serialBuffer[0]; // temperature
            _escTelemetryValues[1] = (_serialBuffer[1] << 8) | _serialBuffer[2]; // voltage
            _escTelemetryValues[2] = (_serialBuffer[3] << 8) | _serialBuffer[4]; // Current
            _escTelemetryValues[3] = (_serialBuffer[5] << 8) | _serialBuffer[6]; // used mA/h
            _escTelemetryValues[4] = (_serialBuffer[7] << 8) | _serialBuffer[8]; // eRpM *100

            _w0_Temperature = (float) _escTelemetryValues[0];
            _w0_Voltage = (float) _escTelemetryValues[1] / 100;
            _w0_Current = (float) _escTelemetryValues[2] / 100;
            _w0_UsedMah = (float) _escTelemetryValues[3];
            _w0_Rpm = (float) _escTelemetryValues[4] * 1000;

            //DisplayText(0.1f, "temp: " + String(_escTelemetryValues[0]), "used mA/h: " + String(_escTelemetryValues[3]));
            _line5 = "T:" + String(_escTelemetryValues[0]) + "C, RPM:" + String(_escTelemetryValues[4]);

        }
    }
    else
    {
        //DisplayText(0.1f, "failed read", "not enough bytes");
        _line5 = "no read, bytes: " + String(_receivedBytes);
    }

    ulong microsAfterDecodeResponse = micros();


    ulong microsAtEnd = micros();

    int discardedBytes = 0;
    while (Serial0.available())
    {
        Serial0.read();
        discardedBytes++;
        _totalDiscardedBytes++;
    }

    //microsAtStart, microsAfterSendSignal, microsAfterReceiveSignal, microsAfterDecodeResponse, microsAtEnd

    /*
    DisplayText(0.01, "Total: " + String((microsAtEnd-microsAtStart)/1000) + "ms",
                "Set Sig: " + String((microsAfterSendSignal-microsAtStart)/1000) + "ms",
                "Get Sig: " + String((microsAfterReceiveSignal-microsAfterSendSignal)/1000) + "ms",
                "Decode: " + String((microsAfterDecodeResponse-microsAfterReceiveSignal)/1000) + "ms",
                "Reset Sig: " + String((microsAtEnd-microsAfterDecodeResponse)/1000) + "ms");
    */

    //Serial.println("Requested Telemetrie");
    //Serial.print("Temperature (C): ");
    //Serial.println(_escTelemetryValues[0]);
    //Serial.print("Voltage: (V) /100: ");
    //Serial.println(_escTelemetryValues[1]);
    //Serial.print("Current (A) /100: ");
    //Serial.println(_escTelemetryValues[2]);
    //Serial.print("used mA/h: ");
    //Serial.println(_escTelemetryValues[3]);
    //Serial.print("eRpM *100: ");
    //Serial.println(_escTelemetryValues[4]);

    //SafeSerialPrintLn("Total: " + String((microsAtEnd-microsAtStart)) + "us, Get reply: "  + String((microsAfterReceiveSignal-_timeTelemetrySignalSentMicros)) + "us, " + _line5 + ", discarded " + String(discardedBytes) + " bytes, total discarded "+ String(_totalDiscardedBytes) + " bytes");

}


uint8_t update_crc8(uint8_t crc, uint8_t crc_seed)
{
    uint8_t crc_u, i;
    crc_u = crc;
    crc_u ^= crc_seed;
    for (i = 0; i < 8; i++)
    { crc_u = (crc_u & 0x80) ? 0x7 ^ (crc_u << 1) : (crc_u << 1); }
    return (crc_u);
}

uint8_t get_crc8(uint8_t *Buf, uint8_t BufLen)
{
    uint8_t crc = 0, i;
    for (i = 0; i < BufLen; i++)
    { crc = update_crc8(Buf[i], crc); }
    return (crc);
}

// outputs

long GetAverageCyclePeriodMilliseconds()
{
    long counter = 0;

    for (int i = 0; i < TIMING_MEASUREMENT_SAMPLES; i++)
    {
        counter += _timingMeasurementBuffer[i];
    }

    return counter / TIMING_MEASUREMENT_SAMPLES;
}

void SetTimingData()
{
    _timingMeasurementBuffer[_currentTimingMeasurementBufferIndex] = micros() - _lastCycleTime;

    // iterate or reset
    _currentTimingMeasurementBufferIndex++;
    if (_currentTimingMeasurementBufferIndex >= TIMING_MEASUREMENT_SAMPLES)
    {
        _currentTimingMeasurementBufferIndex = 0;
    }

    _lastCycleTime = micros();
    long averageCyclePeriod = GetAverageCyclePeriodMilliseconds();

    String updateRateText =
            "Update rate: " + String(1000000 / (float) (averageCyclePeriod)) + " Hz (" +
            String(averageCyclePeriod - 150) +
            "us)";
    _line3 = updateRateText;

    //SafeSerialPrintLn(updateRateText);
}

int ClampInt(int input, int min, int max)
{
    if (input > max)
    {
        return max;
    }
    if (input < min)
    {
        return min;
    }

    return input;
}

int ClampServoAngle(int input)
{
    int DEADZONE_ANGLE = 3;
    if (input >= (90 - DEADZONE_ANGLE) && input <= (90 + DEADZONE_ANGLE))
    {
        input = 90;
    }

    return ClampInt(input, 0, 180);
}

float MapFloatFromByte(int8_t input, float min, float max)
{
    float fraction = ((float) input + (float) 128) / (float) 256;
    float delta = max - min;
    return (float) (delta * fraction + min);
}

int MapIntFromByte(int8_t input, int min, int max)
{
    float fraction = ((float) input + (float) 128) / (float) 256;
    float delta = max - min;
    return (int) (delta * fraction + min);
}

void SetMotorOutputs()
{
    long motorMillisDelta = millis() - _millisSinceLastMotorLoop;

    _millisSinceLastMotorLoop = millis();

    int smartThrottleLeftDrive = 90;
    int smartThrottleRightDrive = 90;

    bool isSmartTurning = false;
    float absAngleDifferenceFromSmartTurning = 0;

    if (_buttonDUPressed == true ||
        _buttonDDPressed == true ||
        _buttonDLPressed == true ||
        _buttonDRPressed == true)
    {
        if (_buttonDUPressed)
        {
            smartThrottleLeftDrive = 180;
            smartThrottleRightDrive = 180;
        }
        if (_buttonDDPressed)
        {
            smartThrottleLeftDrive = 0;
            smartThrottleRightDrive = 0;
        }
        if (_buttonDLPressed)
        {
            smartThrottleLeftDrive += -45;
            smartThrottleRightDrive += 45;
        }
        if (_buttonDRPressed)
        {
            smartThrottleLeftDrive += 45;
            smartThrottleRightDrive += -45;
        }
    }
    else
    {
        _headingResetEngaged = _buttonYPressed;

        int _headingOffset = 0;
        if (_isInverted)
        {
            if (_isInverted)
            {
                _headingOffset = 180;
            }
        }
        auto angleDelta = GetAngleDeltaDegrees(GetOffsetHeading(), _smartHeadingTarget + _headingOffset);

        int throttleToAdd = 10;

        //SafeSerialPrintLn("currentheading: " + String(_orientationYaw) + ", offset: " + String(_smartHeadingOffset) +
        //                  ", target heading: " + String(_smartHeadingTarget) +
        //                  ", new heading:" + String(GetOffsetHeading()) + ", Delta: " +
        //                  String(angleDelta));

        throttleToAdd = abs(MINIMUM_TURNING_THROTTLE + (int) ((float) abs(angleDelta) * _turningMultiplier));

        float invertedMultiplier = 1;
        if (_isInverted)
        {
            invertedMultiplier = -1;
        }

        if (angleDelta > _angleTolerance && _joystickREngaged)
        {
            // turn right
            smartThrottleLeftDrive = 90 + throttleToAdd + TURN_FORWARD_BIAS;
            smartThrottleRightDrive = 90 - throttleToAdd;

            smartThrottleLeftDrive = ClampServoAngle(smartThrottleLeftDrive);
            smartThrottleRightDrive = ClampServoAngle(smartThrottleRightDrive);

            isSmartTurning = true;

            //SafeSerialPrintLn("Throttles: " + String(smartThrottleLeftDrive) + " " + String(smartThrottleRightDrive) + "(Right)");
        }
        else
        {
            if (angleDelta < -_angleTolerance && _joystickREngaged)
            {
                // turn left
                smartThrottleLeftDrive = 90 - throttleToAdd;
                smartThrottleRightDrive = 90 + throttleToAdd + TURN_FORWARD_BIAS;

                smartThrottleLeftDrive = ClampServoAngle(smartThrottleLeftDrive);
                smartThrottleRightDrive = ClampServoAngle(smartThrottleRightDrive);

                isSmartTurning = true;

                //SafeSerialPrintLn("Throttles: " + String(smartThrottleLeftDrive) + " " + String(smartThrottleRightDrive) + "(Left)");
            }
            else
            {
                // neutral
                smartThrottleLeftDrive = 90;
                smartThrottleRightDrive = 90;

                //SafeSerialPrintLn("Throttles: " + String(smartThrottleLeftDrive) + " " + String(smartThrottleRightDrive) + "(Neutral)");
            }
        }

        smartThrottleLeftDrive += _smartThrottleDrive * _additiveThrottleMultiplier * invertedMultiplier;
        smartThrottleRightDrive += _smartThrottleDrive * _additiveThrottleMultiplier * invertedMultiplier;

        absAngleDifferenceFromSmartTurning = abs(angleDelta);
    }


    smartThrottleLeftDrive = ClampServoAngle(smartThrottleLeftDrive);
    smartThrottleRightDrive = ClampServoAngle(smartThrottleRightDrive);


    /*
    SetMotorOutput(_testServoL, _throttleLeftDrive);
    SetMotorOutput(_testServoR, _throttleRightDrive);
     */

    float rampupDegreesPerSecond = 90.0 / _rampupTime;

    float degreesToChange = rampupDegreesPerSecond * motorMillisDelta / 1000;

    _currentSmartThrottleWeapon1 = 180 - _throttleWeapon1;

    if (isSmartTurning)
    {
        auto lowCeiling = absAngleDifferenceFromSmartTurning * _attenuationWeaponThrottleTurning;
        auto highCeiling = 180 - (absAngleDifferenceFromSmartTurning * _attenuationWeaponThrottleTurning);
        auto marginFromZero = 20;

        if (lowCeiling > (90 - marginFromZero))
        {
            lowCeiling = 90 - marginFromZero;
        }

        if (highCeiling < (90 + marginFromZero))
        {
            highCeiling = 90 + marginFromZero;
        }


        if (_currentSmoothThrottleWeapon1 > highCeiling)
        {
            _currentSmoothThrottleWeapon1 = highCeiling;
        }

        if (_currentSmoothThrottleWeapon1 < lowCeiling)
        {
            _currentSmoothThrottleWeapon1 = lowCeiling;
        }
    }


    if (_isTilted)
    {
        if (_currentSmoothThrottleWeapon1 > 120)
        {
            _currentSmoothThrottleWeapon1 = 120;
        }

        if (_currentSmoothThrottleWeapon1 < 60)
        {
            _currentSmoothThrottleWeapon1 = 60;
        }
    }


    if (_currentSmoothThrottleWeapon1 > _currentSmartThrottleWeapon1)
    {
        // decrease
        if (_currentSmartThrottleWeapon1 < 90)
        {
            _currentSmoothThrottleWeapon1 -= degreesToChange;
        }
        else
        {
            if (_currentSmoothThrottleWeapon1 < 93)
            {
                _currentSmoothThrottleWeapon1 = 90;
            }
            else
            {
                if (_currentSmoothThrottleWeapon1 > 93 || _isTilted || isSmartTurning > _attenuationWeaponThrottle)
                {
                    _currentSmoothThrottleWeapon1 -= degreesToChange * 4 * _rampupTime;
                }
            }

        }
    }
    else
    {
        // increase
        if (_currentSmartThrottleWeapon1 > 90)
        {
            _currentSmoothThrottleWeapon1 += degreesToChange;
        }
        else
        {
            if (_currentSmoothThrottleWeapon1 > 87)
            {
                _currentSmoothThrottleWeapon1 = 90;
            }
            else
            {
                if (_currentSmoothThrottleWeapon1 < 87 || _isTilted || isSmartTurning > _attenuationWeaponThrottle)
                {
                    _currentSmoothThrottleWeapon1 += degreesToChange * 4 * _rampupTime;
                }
            }

        }
    }


    if ((_bluetoothClientExists && _receivedHeartbeatFromClient) || _pwmTimingDebugEnabled)
    {
        EnableMotors();
    }
    else
    {
        DisableMotors();
    }


#if MOTOR_L_USES_DSHOT
    //motorLeftDrive.sendThrottle3D(DegreesToDShotThrottle(smartThrottleLeftDrive));
    int16_t milliswrap = sin(millis()*2/10000*PI)*50;
    motorLeftDrive.sendThrottle3D(milliswrap);
    SafeSerialPrintLn("new throttle: " + String(milliswrap));
    delay(1);
#else
    allServos.write(PIN_TEST_SERVO_L, smartThrottleLeftDrive);
#endif
    _currentSmartThrottleLeftDrive = smartThrottleLeftDrive;


#if MOTOR_R_USES_DSHOT
    motorRightDrive.sendThrottle3D(DegreesToDShotThrottle(smartThrottleRightDrive));
#else
    allServos.write(PIN_TEST_SERVO_R, smartThrottleRightDrive);
#endif
    _currentSmartThrottleRightDrive = smartThrottleRightDrive;


#if MOTOR_W1_USES_DSHOT
    motorWeapon1.sendThrottle3D(DegreesToDShotThrottle(_currentSmoothThrottleWeapon1));
#else
    if (_currentSmoothThrottleWeapon1 > 87 && _currentSmoothThrottleWeapon1 < 93)
    {
        allServos.write(PIN_TEST_SERVO_W1, 90);
    }
    else
    {
        allServos.write(PIN_TEST_SERVO_W1, _currentSmoothThrottleWeapon1);
    }
#endif


    int clampedThrottle = 0;
    if (_throttleWeapon0 > 45)
    {
        clampedThrottle = 90;
    }
#if MOTOR_W0_USES_DSHOT
    motorWeapon0.sendThrottleValue(DegreesToDShotThrottle(clampedThrottle));
#else
    allServos.write(PIN_TEST_SERVO_W0, 180 - 2 * clampedThrottle);
#endif


    /*
    if(_timeSinceLastPwmUpdateUs == 0)
    {
        _timeSinceLastPwmUpdateUs = micros();
    }

    //delayMicroseconds(REFRESH_INTERVAL/2);

    long currentTimeUs = micros();
    long timeDifferenceSinceLastUpdateUs = currentTimeUs - _timeSinceLastPwmUpdateUs;
    long addedDelay = (REFRESH_INTERVAL-timeDifferenceSinceLastUpdateUs);

    SafeSerialPrintLn("time difference: " + String(timeDifferenceSinceLastUpdateUs) + " catchup time: " + String(addedDelay));

    while(addedDelay < 0)
    {
        SafeSerialPrintLn("added delay was negative: " + String(addedDelay));
        addedDelay += REFRESH_INTERVAL;
    }
    delayMicroseconds(addedDelay);
*/
    _timeSinceLastPwmUpdateUs = micros();
}

uint16_t DegreesToDShotThrottle(int degreesInput)
{
    return (degreesInput) * 1000 / (180);
}

void SetLeds()
{
#if LEDS_ENABLED == false
    return;
#endif


    if ((_bluetoothClientExists && _receivedHeartbeatFromClient) || GetFlashValue(500, false))
    {
        if (_voltageReadingMillivolts > MINIMUM_VOLTAGE_BATTERY_FULL)
        {
            SetMainLeds(_colorGreen);
        }
        else
        {
            if (_voltageReadingMillivolts > MINIMUM_VOLTAGE_BATTERY_HIGH)
            {
                SetMainLeds(_colorGreen);
            }
            else
            {
                if (_voltageReadingMillivolts > MINIMUM_VOLTAGE_BATTERY_LOW)
                {
                    SetMainLeds(_colorYellow);

                }
                else
                {
                    if (_voltageReadingMillivolts > MINIMUM_VOLTAGE_BATTERY_DEAD)
                    {
                        SetMainLeds(_colorRed);
                    }
                    else
                    {
                        if (_voltageReadingMillivolts > MINIMUM_VOLTAGE_USING_USB)
                        {
                            if (GetFlashValue(250, true))
                            {
                                SetMainLeds(_colorRed);
                            }
                            else
                            {
                                SetMainLeds(_colorBlack);
                            }
                        }
                        else
                        {
                            SetMainLeds(_colorPurple);
                        }
                    }
                }
            }
        }


    }
    else
    {
        if (GetFlashValue(500, true))
        {
            SetMainLeds(_colorBlue);
        }
    }

    /*
    for (int i = 2; i < 8; i++)
    {
        if (i == _wheelIndex)
        {
            _leds[i] = _colorWhite;
        }
        else
        {
            _leds[i] = _colorBlack;
        }
    }

    _wheelIndex++;
    if (_wheelIndex == 8)
    {
        _wheelIndex = 2;
    }
     */

    uint32_t _colorLeftDrive;
    uint32_t _colorRightDrive;
    uint32_t _colorWeapon;
    uint32_t _colorWeaponMax;
    uint32_t _colorTilt;


    if (_currentSmartThrottleLeftDrive > 93)
    {
        int16_t throttleAbsoluteStrength = _currentSmartThrottleLeftDrive - 90;
        _colorLeftDrive = strip.ColorHSV(HUE_FORWARD, SAT_FORWARD, 40 + throttleAbsoluteStrength);

    }
    else
    {
        if (_currentSmartThrottleLeftDrive < 87)
        {
            int16_t throttleAbsoluteStrength = 90 - _currentSmartThrottleLeftDrive;
            _colorLeftDrive = strip.ColorHSV(HUE_REVERSE, SAT_REVERSE, 40 + throttleAbsoluteStrength);
        }
        else
        {
            _colorLeftDrive = _colorPurple;
        }
    }


    if (_currentSmartThrottleRightDrive > 93)
    {
        int16_t throttleAbsoluteStrength = _currentSmartThrottleRightDrive - 90;
        _colorRightDrive = strip.ColorHSV(HUE_FORWARD, SAT_FORWARD, 40 + throttleAbsoluteStrength);

    }
    else
    {
        if (_currentSmartThrottleRightDrive < 87)
        {
            int16_t throttleAbsoluteStrength = 90 - _currentSmartThrottleRightDrive;
            _colorRightDrive = strip.ColorHSV(HUE_REVERSE, SAT_REVERSE, 40 + throttleAbsoluteStrength);
        }
        else
        {
            _colorRightDrive = _colorPurple;
        }
    }


    if (_currentSmoothThrottleWeapon1 > 93)
    {
        int16_t throttleAbsoluteStrength = _currentSmoothThrottleWeapon1 - 90;
        _colorWeapon = strip.ColorHSV(HUE_REVERSE, HUE_REVERSE, 40 + throttleAbsoluteStrength);

    }
    else
    {
        if (_currentSmoothThrottleWeapon1 < 87)
        {
            int16_t throttleAbsoluteStrength = 90 - _currentSmoothThrottleWeapon1;
            _colorWeapon = strip.ColorHSV(HUE_FORWARD, SAT_FORWARD, 40 + throttleAbsoluteStrength);
        }
        else
        {
            _colorWeapon = _colorPurple;
        }
    }

    _colorWeaponMax = strip.ColorHSV(HUE_FORWARD, SAT_FORWARD, 255.0 * _attenuationWeaponThrottle);
    if (_attenuationWeaponThrottle > 0.95)
    {
        _colorWeaponMax = strip.ColorHSV(HUE_REVERSE, HUE_REVERSE, 255);
    }

    if (_isTilted)
    {
        _colorTilt = strip.ColorHSV(HUE_REVERSE, HUE_REVERSE, 255);
    }
    else
    {
        _colorTilt = strip.ColorHSV(HUE_REVERSE, HUE_REVERSE, 0);
    }

    strip.setPixelColor(LED_L_DRIVE, _colorLeftDrive);
    strip.setPixelColor(LED_R_DRIVE, _colorRightDrive);
    strip.setPixelColor(LED_W1, _colorWeapon);
    strip.setPixelColor(LED_TILT, _colorTilt);


    strip.setPixelColor(3, _colorWeaponMax);
    strip.setPixelColor(7, _colorWeaponMax);

    /*
    strip.setPixelColor(0, strip.Color(255,255,255));
    strip.setPixelColor(1, strip.Color(255,255,255));
    strip.setPixelColor(2, strip.Color(255,0,0));
    strip.setPixelColor(3, strip.Color(255,255,0));
    strip.setPixelColor(4, strip.Color(0,255,0));
    strip.setPixelColor(5, strip.Color(0,255,255));
    strip.setPixelColor(6, strip.Color(0,0,255));
    strip.setPixelColor(7, strip.Color(255,0,255));
    */

    strip.show();
}

void SetMainLeds(uint32_t color)
{
    strip.setPixelColor(LED_BOARD, color);
    strip.setPixelColor(LED_CENTER, color);
}

bool GetFlashValue(int periodMilliseconds, bool startsTrue)
{
    if (startsTrue)
    {
        return (millis() % periodMilliseconds < periodMilliseconds / 2);
    }
    else
    {
        return (millis() % periodMilliseconds > periodMilliseconds / 2);
    }

}

void SetFailsafe()
{
    if (millis() - _timeLastReceivedHeartbeatMillis > TIMEOUT_HEARTBEAT_LOST)
    {
        _receivedHeartbeatFromClient = false;

        if (millis() - _timeLastReceivedHeartbeatMillis > TIMEOUT_HEARTBEAT_LOST_REBOOT)
        {
            esp_restart();
        }
    }
    else
    {
        _receivedHeartbeatFromClient = true;
    }
}

void RestartOnButtonCombination()
{
    if (_buttonAPressed && _buttonBPressed && _buttonXPressed && _buttonYPressed)
    {
        esp_restart();
    }
}

void PushFloatToTelemetryVector(float floatValue)
{
    unsigned char const *p = reinterpret_cast<unsigned char const *>(&floatValue);
    _telemetryVector.push_back(p[0]);
    _telemetryVector.push_back(p[1]);
    _telemetryVector.push_back(p[2]);
    _telemetryVector.push_back(p[3]);
}

void PushIntSixteenToTelemetryVector(int16_t intValue)
{
    unsigned char const *p = reinterpret_cast<unsigned char const *>(&intValue);
    _telemetryVector.push_back(p[0]);
    _telemetryVector.push_back(p[1]);
}

void GetAndSetBluetoothData()
{
    _telemetryVector.clear();

    if (pServer->getConnectedCount())
    {
        _bluetoothClientExists = true;

        NimBLEService *pSvc = pServer->getServiceByUUID(SERVICE_UUID);
        if (pSvc)
        {
            NimBLECharacteristic *pChrAll = pSvc->getCharacteristic(TELEMETRY_ALL_UUID);
            if (pChrAll)
            {
                PushIntSixteenToTelemetryVector(_voltageReadingMillivolts);

                PushFloatToTelemetryVector(_orientationR);
                PushFloatToTelemetryVector(_orientationI);
                PushFloatToTelemetryVector(_orientationJ);
                PushFloatToTelemetryVector(_orientationK);

                PushIntSixteenToTelemetryVector(_bnoAccelerationX * 1000);
                PushIntSixteenToTelemetryVector(_bnoAccelerationY * 1000);
                PushIntSixteenToTelemetryVector(_bnoAccelerationZ * 1000);

                PushIntSixteenToTelemetryVector(_lisAccelerationX);
                PushIntSixteenToTelemetryVector(_lisAccelerationY);
                PushIntSixteenToTelemetryVector(_lisAccelerationZ);

                PushIntSixteenToTelemetryVector(_w0_Temperature);
                PushIntSixteenToTelemetryVector(_w0_Voltage);
                PushIntSixteenToTelemetryVector(_w0_Current);
                PushIntSixteenToTelemetryVector(_w0_UsedMah);
                PushIntSixteenToTelemetryVector(_w0_Rpm);

                _telemetryVector.push_back(_bnoTemp);

                _telemetryVector.push_back(_bnoCalibrationSystem);
                _telemetryVector.push_back(_bnoCalibrationGyro);
                _telemetryVector.push_back(_bnoCalibrationAccelerometer);
                _telemetryVector.push_back(_bnoCalibrationMagnetometer);

                PushIntSixteenToTelemetryVector(_smartHeadingOffset);


                pChrAll->setValue(_telemetryVector);
                pChrAll->notify(true);
            }
            NimBLECharacteristic *pCtrlL = pSvc->getCharacteristic(CTRL_ALL_UUID);
            if (pCtrlL)
            {
                _controlMessage = pCtrlL->getValue();

                if (_controlMessage != "PEE")
                {
                    uint8_t *pData = (uint8_t *) _controlMessage.data();
                    /*
                    byte ctrlByteArray[8];
                    ctrlByteArray[0] =  ctrlAsInt &   0x00000000000000ff;
                    ctrlByteArray[1] = (ctrlAsInt &   0x000000000000ff00) >> 8;
                    ctrlByteArray[2] =  (ctrlAsInt &  0x0000000000ff0000) >> 16;
                    ctrlByteArray[3] = (ctrlAsInt &   0x00000000ff000000) >> 24;
                    ctrlByteArray[4] = (ctrlAsInt &   0x000000ff00000000) >> 32;
                    ctrlByteArray[5] =  (ctrlAsInt &  0x0000ff0000000000) >> 40;
                    ctrlByteArray[6] = (ctrlAsInt &   0x00ff000000000000) >> 48;
                    ctrlByteArray[7] = (ctrlAsInt &   0xff00000000000000) >> 56;
                     */

                    short securityByteStart = (pData[1] << 8) | pData[0];
                    short securityByteEnd = (pData[37] << 8) | pData[36];

                    _passesSecurityByteValidation = (securityByteStart == _securityByteValidation) &&
                                                    (securityByteEnd == _securityByteValidation);

                    if (_passesSecurityByteValidation)
                    {
                        long heartbeatTime = (pData[35] << 24) | (pData[34] << 16) | (pData[33] << 8) | (pData[32]);

                        _previousHeartbeatTimeMillis = _currentHeartbeatTimeMillis;
                        _currentHeartbeatTimeMillis = heartbeatTime;

                        if (_currentHeartbeatTimeMillis > _previousHeartbeatTimeMillis)
                        {
                            short buttonValues = (pData[3] << 8) | pData[2];

                            _buttonAPressed = CHECK_BIT(buttonValues, 15);
                            _buttonBPressed = CHECK_BIT(buttonValues, 14);
                            _buttonXPressed = CHECK_BIT(buttonValues, 13);
                            _buttonYPressed = CHECK_BIT(buttonValues, 12);
                            _buttonDUPressed = CHECK_BIT(buttonValues, 11);
                            _buttonDDPressed = CHECK_BIT(buttonValues, 10);
                            _buttonDLPressed = CHECK_BIT(buttonValues, 9);
                            _buttonDRPressed = CHECK_BIT(buttonValues, 8);
                            _buttonL1Pressed = CHECK_BIT(buttonValues, 7);
                            _buttonR1Pressed = CHECK_BIT(buttonValues, 6);
                            _joystickLEngaged = CHECK_BIT(buttonValues, 5);
                            _joystickREngaged = CHECK_BIT(buttonValues, 4);


                            _angleTolerance = MapIntFromByte(_settingAngleTolerance, _settingAngleToleranceMin,
                                                             _settingAngleToleranceMax);
                            _turningMultiplier = MapFloatFromByte(_settingTurningMultiplier,
                                                                  _settingTurningMultiplierMin,
                                                                  _settingTurningMultiplierMax);
                            _additiveThrottleMultiplier = MapFloatFromByte(_settingAdditiveThrottleMultiplier,
                                                                           _settingAdditiveThrottleMultiplierMin,
                                                                           _settingAdditiveThrottleMultiplierMax);

                            _attenuationWeaponThrottle = MapFloatFromByte(_settingAttenuationWeaponThrottle,
                                                                          _settingAttenuationWeaponThrottleMin,
                                                                          _settingAttenuationWeaponThrottleMax);

                            _attenuationWeaponThrottleTurning = MapFloatFromByte(
                                    _settingAttenuationWeaponThrottleTurning,
                                    _settingAttenuationWeaponThrottleTurningMin,
                                    _settingAttenuationWeaponThrottleTurningMax);


                            //SafeSerialPrintLn(
                            //        "weapon attenuation angle: " + String(_attenuationWeaponThrottle));

                            short possibleHeadingValue = (pData[5] << 8) | pData[4];

                            if (!_buttonYPressed && _joystickREngaged)
                            {
                                _smartHeadingTarget = possibleHeadingValue;
                            }


                            _smartThrottleDrive = (pData[7] << 8) | (pData[6]);

                            _throttleLeftDrive = (pData[9] << 8) | pData[8];
                            _throttleRightDrive = (pData[11] << 8) | (pData[10]);
                            _throttleWeapon0 = (pData[13] << 8) | pData[12];
                            _throttleWeapon1 = (pData[15] << 8) | (pData[14]);

                            _settingAngleTolerance = pData[16];
                            _settingTurningMultiplier = pData[17];
                            _settingAdditiveThrottleMultiplier = pData[18];
                            _settingAttenuationWeaponThrottle = pData[19];
                            _settingAttenuationWeaponThrottleTurning = pData[20];


                            _skippedHeartbeats = 0;
                            _timeLastReceivedHeartbeatMillis = millis();
                            //SafeSerialPrintLn(
                            //        "ctrl: " + String(securityByteStart) + " " + String(possibleHeadingValue) + " " +
                            //        String(buttonValues) + " " +
                            //        String(_joystickREngaged) + " " + String(_headingResetEngaged) + " " +
                            //        String(_smartThrottleDrive) + " " + String(_throttleRightDrive) + " " +
                            //        String(_throttleLeftDrive) + " " + String(_throttleWeapon0) + " " +
                            //        String(_throttleWeapon1) + " " + String(_currentHeartbeatTimeMillis) + " " +
                            //        String(securityByteEnd) + " ");
                        }

                        //SafeSerialPrintLn(
                        //        "INVALID, NO INCREASE HEARTBEAT. ctrl: " + String(securityByteStart) + " "+ String(_throttleLeftDrive) + " " + String(_throttleRightDrive) + " " + String(_throttleWeapon0) + " " +
                        //        String(_throttleWeapon1) + " " + String(_currentHeartbeatTimeMillis) + " "+ String(securityByteEnd) + " ");

                        _skippedHeartbeats++;
                    }
                    else
                    {
                        SafeSerialPrintLn(
                                "INVALID, BAD SEC BYTE ctrl: " + String(securityByteStart) + " " +
                                String(_throttleLeftDrive) + " " + String(_throttleRightDrive) + " " +
                                String(_throttleWeapon0) + " " +
                                String(_throttleWeapon1) + " " + String(_currentHeartbeatTimeMillis) + " " +
                                String(securityByteEnd) + " ");

                        _skippedHeartbeats++;
                    }


                }

            }
        }
    }
    else
    {
        _bluetoothClientExists = false;
        _skippedHeartbeats++;
    }
}

// extensions

bool isNumber(const std::string &s)
{
    int counter = 0;

    for (char const &ch: s)
    {
        counter++;
        if (std::isdigit(ch) == 0)
        {
            return false;
        }
    }

    if (counter == 0)
    {
        return false;
    }
    return true;
}

void SafeSerialPrintLn(String lineToPrint)
{
    if (_serialDebugEnabled && Serial.availableForWrite())
    {
        Serial.println(lineToPrint);
    }
}

void DisplayText(float timeSeconds, String line0, String line1, String line2, String line3, String line4, String line5)
{
    if (!_displayEnabledAndFound)
    {
        return;
    }

    Display.clearDisplay();
    Display.setTextSize(1);
    Display.setTextColor(WHITE);

    Display.setCursor(0, 0);
    Display.println(line0);

    Display.setCursor(0, 10);
    Display.println(line1);

    Display.setCursor(0, 20);
    Display.println(line2);

    Display.setCursor(0, 30);
    Display.println(line3);

    Display.setCursor(0, 40);
    Display.println(line4);

    Display.setCursor(0, 50);
    Display.println(line5);


    Display.display();

    if (_debugEnabled)
    {
        delay(1000 * timeSeconds);
    }
    else
    {
        delay(100 * timeSeconds);
    }
}
