#include <main.h>

// lifecycle

void setup()
{
    Serial.begin(115200);
    Serial.setTimeout(1);
    SafeSerialPrintLn("Beginning serial");
    Wire.begin();
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
    InitializeMotors();
#endif

#if TELEMETRY_ENABLED
    InitializeEscTelemetry();
#endif


}

void loop()
{
    delayMicroseconds(10); // this is just in for safety...


#if TELEMETRY_ENABLED
    // must be done at the start to give the most time for a response signal to come back
    SetWeaponTelemetrySignal();
#endif

    while ((micros() - timeTelemetrySignalSentMicros) < DELAY_BEFORE_TELEMETRY_COMES_BACK)
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
#if (USING_LEGACY_BNO055 != true)
    if (!bno.begin_I2C())
    {
        DisplayText(1, "IMU Error", "Bad Init");
        _imuEnabledAndFound = true;
    }
    else
    {
        DisplayText(0.25, "IMU Success");
        bno.enableReport(SH2_GAME_ROTATION_VECTOR);
        _imuEnabledAndFound = false;
    }
#else
    if (!bno.begin(OPERATION_MODE_IMUPLUS))
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
    }
    else
    {
        DisplayText(0.25, "Accel Success");
        lis.setRange(H3LIS331_RANGE_400_G);
        _accelerometerEnabledAndFound = true;
    }
}

void InitializeLeds()
{
    pinMode(PIN_NUM_NEOPIXEL_OUTPUT, OUTPUT);
    delay(1);
    FastLED.addLeds<NEOPIXEL, PIN_NUM_NEOPIXEL_OUTPUT>(leds, TOTAL_LED);
    FastLED.show();
}

void InitializeBluetooth()
{
    NimBLEDevice::init(BLEName);
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

void InitializeMotors()
{
    DisplayText(0.25, "Attaching motors", "Pins: " + String(PIN_TEST_SERVO_L) + ", " + String(PIN_TEST_SERVO_R));

    ESP32_ISR_Servos.useTimer(USE_ESP32_TIMER_NO);

    ESP32_ISR_Servos.disableAll();

    servoLIndex = ESP32_ISR_Servos.setupServo(PIN_TEST_SERVO_L, 1000, 2000);
    servoRIndex = ESP32_ISR_Servos.setupServo(PIN_TEST_SERVO_R, 1000, 2000);
    servoW0Index = ESP32_ISR_Servos.setupServo(PIN_TEST_SERVO_W0, 0, 2000);
    servoW1Index = ESP32_ISR_Servos.setupServo(PIN_TEST_SERVO_W1, 1000, 2000);

    if (servoLIndex == -1)
    {
        DisplayText(1, "Failure L Servo");
    }
    else
    {
        DisplayText(0.25, "Success L Servo");
    }

    if (servoRIndex == -1)
    {
        DisplayText(1, "Failure R Servo");
    }
    else
    {
        DisplayText(0.25, "Success R Servo");
    }

    if (servoW0Index == -1)
    {
        DisplayText(1, "Failure W0 Servo");
    }
    else
    {
        DisplayText(0.25, "Success W0 Servo");
    }

    if (servoW1Index == -1)
    {
        DisplayText(1, "Failure W1 Servo");
    }
    else
    {
        DisplayText(0.25, "Success W1 Servo");
    }

}


void InitializeVoltageReader()
{
    pinMode(PIN_VOLTAGE_READER, INPUT_PULLDOWN);
}

// inputs

void GetIMUData()
{
    if (!_imuEnabledAndFound)
    {
        return;
    }
#if USING_LEGACY_BNO055 == false
    

    sh2_SensorValue_t sensorValues;
    bno.getSensorEvent(&sensorValues);

    uint8_t system, gyro, accel, mag;
    system = gyro = accel = mag = 0;

    _line0 = "Orientation";


    _line1 =
            "(" + String(sensorValues.un.gameRotationVector.i) + "," + String(sensorValues.un.gameRotationVector.j) +
            "," +
            String(sensorValues.un.gameRotationVector.k) + "," + String(sensorValues.un.gameRotationVector.real) + ")";

    orientationR = sensorValues.un.gameRotationVector.real;
    orientationI = sensorValues.un.gameRotationVector.i;
    orientationJ = sensorValues.un.gameRotationVector.j;
    orientationK = sensorValues.un.gameRotationVector.k;
    
    BNOAccelerationX = sensorValues.un.accelerometer.x;
    BNOAccelerationY = sensorValues.un.accelerometer.y;
    BNOAccelerationZ = sensorValues.un.accelerometer.z;
    
#else
    bno.getEvent(&event);

    imu::Quaternion quat = bno.getQuat();

    orientationR = quat.w();
    orientationI = quat.x();
    orientationJ = quat.y();
    orientationK = quat.z();

    BNOAccelerationX = event.acceleration.x;
    BNOAccelerationY = event.acceleration.y;
    BNOAccelerationZ = event.acceleration.z;
    
#endif

}

void GetAccelerometerData()
{
    if (!_accelerometerEnabledAndFound)
    {
        return;
    }
    
    lis.getEvent(&event);

    LISAccelerationX = event.acceleration.x;
    LISAccelerationY = event.acceleration.y;
    LISAccelerationZ = event.acceleration.z;
}

void GetVoltageData()
{
    voltageReadingRaw = analogRead(PIN_VOLTAGE_READER);
    // batt is 15.38, v after divider is 1.392, raw is
    voltageReadingMv = (int16_t) ((float) (voltageReadingRaw) * (7.907455));
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

    ESP32_ISR_Servos.setPulseWidth(servoW0Index, telemetryRequestWidth);
    timeTelemetrySignalSentMicros = micros();

    delayMicroseconds(REFRESH_INTERVAL);
}

void GetWeaponTelemetry()
{

    receivedBytes = 0;

    //read in telemetry from serial
    ulong microsAtStart = micros();

    while ((micros() - timeTelemetrySignalSentMicros < (TELEMETRY_READ_TIMEOUT)) && receivedBytes < 10)
    {
        if (Serial0.available())
        {
            SerialBuf[receivedBytes] = Serial0.read();
            receivedBytes++;
        }

    }

    ulong microsAfterReceiveSignal = micros();

    if (receivedBytes >= 9)
    {
        uint8_t crc8 = get_crc8(SerialBuf, 9); // get the 8 bit CRC

        if (crc8 != SerialBuf[9])
        {
            //DisplayText(0.1f, "failed read", "corrupt data");
            _line5 = "corrupt read";
        }
        else
        {
            ESC_telemetrie[0] = SerialBuf[0]; // temperature
            ESC_telemetrie[1] = (SerialBuf[1] << 8) | SerialBuf[2]; // voltage
            ESC_telemetrie[2] = (SerialBuf[3] << 8) | SerialBuf[4]; // Current
            ESC_telemetrie[3] = (SerialBuf[5] << 8) | SerialBuf[6]; // used mA/h
            ESC_telemetrie[4] = (SerialBuf[7] << 8) | SerialBuf[8]; // eRpM *100

            W0_Temperature = (float) ESC_telemetrie[0];
            W0_Voltage = (float) ESC_telemetrie[1] / 100;
            W0_Current = (float) ESC_telemetrie[2] / 100;
            W0_UsedMah = (float) ESC_telemetrie[3];
            W0_Rpm = (float) ESC_telemetrie[4] * 1000;

            //DisplayText(0.1f, "temp: " + String(ESC_telemetrie[0]), "used mA/h: " + String(ESC_telemetrie[3]));
            _line5 = "T:" + String(ESC_telemetrie[0]) + "C, RPM:" + String(ESC_telemetrie[4]);

        }
    }
    else
    {
        //DisplayText(0.1f, "failed read", "not enough bytes");
        _line5 = "no read, bytes: " + String(receivedBytes);
    }

    ulong microsAfterDecodeResponse = micros();


    ulong microsAtEnd = micros();

    int discardedBytes = 0;
    while (Serial0.available())
    {
        Serial0.read();
        discardedBytes++;
        totalDiscardedBytes++;
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
    //Serial.println(ESC_telemetrie[0]);
    //Serial.print("Voltage: (V) /100: ");
    //Serial.println(ESC_telemetrie[1]);
    //Serial.print("Current (A) /100: ");
    //Serial.println(ESC_telemetrie[2]);
    //Serial.print("used mA/h: ");
    //Serial.println(ESC_telemetrie[3]);
    //Serial.print("eRpM *100: ");
    //Serial.println(ESC_telemetrie[4]);

    //SafeSerialPrintLn("Total: " + String((microsAtEnd-microsAtStart)) + "us, Get reply: "  + String((microsAfterReceiveSignal-timeTelemetrySignalSentMicros)) + "us, " + _line5 + ", discarded " + String(discardedBytes) + " bytes, total discarded "+ String(totalDiscardedBytes) + " bytes");

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
        counter += timingMeasurementBuffer[i];
    }

    return counter / TIMING_MEASUREMENT_SAMPLES;
}

void SetTimingData()
{
    timingMeasurementBuffer[currentTimingMeasurementBufferIndex] = micros() - lastCycleTime;

    // iterate or reset
    currentTimingMeasurementBufferIndex++;
    if (currentTimingMeasurementBufferIndex >= TIMING_MEASUREMENT_SAMPLES)
    {
        currentTimingMeasurementBufferIndex = 0;
    }

    lastCycleTime = micros();
    long averageCyclePeriod = GetAverageCyclePeriodMilliseconds();

    String updateRateText =
            "Update rate: " + String(1000000 / (float) (averageCyclePeriod)) + " Hz (" +
            String(averageCyclePeriod - 150) +
            "us)";
    _line3 = updateRateText;

    //SafeSerialPrintLn(updateRateText);
}

void SetMotorOutputs()
{
    /*
    SetMotorOutput(_testServoL, ctrlValue0);
    SetMotorOutput(_testServoR, ctrlValue1);
     */

    if (bluetoothClientExists)
    {
        ESP32_ISR_Servos.enableAll();
    }
    else
    {
        ESP32_ISR_Servos.disableAll();
    }


    if (servoLIndex != -1)
    {
        ESP32_ISR_Servos.setPosition(servoLIndex, ctrlValue0);
    }
    if (servoRIndex != -1)
    {
        ESP32_ISR_Servos.setPosition(servoRIndex, ctrlValue1);
    }
    if (servoW1Index != -1)
    {
        ESP32_ISR_Servos.setPosition(servoW1Index, ctrlValue3);
    }
    if (servoW0Index != -1)
    {
        ulong controlPosition = 1000 + ctrlValue2 * (2000 / 180);
        ESP32_ISR_Servos.setPulseWidth(servoW0Index, controlPosition);
    }
    delayMicroseconds(REFRESH_INTERVAL);
}

void SetLeds()
{
#if LEDS_ENABLED == false
    return;
#endif

    /*
    if (GetFlashValue(500))
    {
        SetMainLeds(DC_White);
    }
    else
    {
        SetMainLeds(DC_Grey);
    }
     */

    if (bluetoothClientExists || GetFlashValue(500, false))
    {
        if (voltageReadingMv > MINIMUM_VOLTAGE_BATTERY_FULL)
        {
            SetMainLeds(CRGB::Green);
        }
        else if (voltageReadingMv > MINIMUM_VOLTAGE_BATTERY_LOW)
        {
            SetMainLeds(CRGB::Yellow);
        }
        else if (voltageReadingMv > MINIMUM_VOLTAGE_BATTERY_DEAD)
        {
            SetMainLeds(CRGB::Red);
        }
        else if (voltageReadingMv > MINIMUM_VOLTAGE_USING_USB)
        {
            if (GetFlashValue(250, true))
            {
                SetMainLeds(CRGB::Red);
            }
            else
            {
                SetMainLeds(CRGB::Black);
            }
        }
        else
        {
            SetMainLeds(CRGB::Purple);
        }


    }
    else
    {
        if (GetFlashValue(500, true))
        {
            SetMainLeds(CRGB::Blue);
        }
        else
        {
            SetMainLeds(CRGB::Black);
        }
    }

    for (int i = 2; i < 8; i++)
    {
        if (i == wheelIndex)
        {
            leds[i] = DC_White;
        }
        else
        {
            leds[i] = CRGB::Black;
        }
    }

    wheelIndex++;
    if (wheelIndex == 8)
    {
        wheelIndex = 2;
    }

    FastLED.show();
}

void SetMainLeds(CRGB color)
{
    leds[LED_BOARD] = color;
    leds[LED_CENTER] = color;
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

void PushFloatToTelemetryVector(float floatValue)
{
    unsigned char const * p = reinterpret_cast<unsigned char const *>(&floatValue);
    TelemetryVector.push_back(p[0]);
    TelemetryVector.push_back(p[1]);
    TelemetryVector.push_back(p[2]);
    TelemetryVector.push_back(p[3]);
}

void PushIntSixteenToTelemetryVector(int16_t intValue)
{
    unsigned char const * p = reinterpret_cast<unsigned char const *>(&intValue);
    TelemetryVector.push_back(p[0]);
    TelemetryVector.push_back(p[1]);
}

void GetAndSetBluetoothData()
{
    TelemetryVector.clear();
    
    if (pServer->getConnectedCount())
    {
        bluetoothClientExists = true;

        NimBLEService *pSvc = pServer->getServiceByUUID(SERVICE_UUID);
        if (pSvc)
        {
            NimBLECharacteristic *pChrAll = pSvc->getCharacteristic(TELEMETRY_ALL_UUID);
            if (pChrAll)
            {
                PushIntSixteenToTelemetryVector(voltageReadingMv);
                
                PushFloatToTelemetryVector(orientationR);
                PushFloatToTelemetryVector(orientationI);
                PushFloatToTelemetryVector(orientationJ);
                PushFloatToTelemetryVector(orientationK);

                PushIntSixteenToTelemetryVector(BNOAccelerationX);
                PushIntSixteenToTelemetryVector(BNOAccelerationY);
                PushIntSixteenToTelemetryVector(BNOAccelerationZ);

                PushIntSixteenToTelemetryVector(LISAccelerationX);
                PushIntSixteenToTelemetryVector(LISAccelerationY);
                PushIntSixteenToTelemetryVector(LISAccelerationZ);

                PushIntSixteenToTelemetryVector(W0_Temperature);
                PushIntSixteenToTelemetryVector(W0_Voltage);
                PushIntSixteenToTelemetryVector(W0_Current);
                PushIntSixteenToTelemetryVector(W0_UsedMah);
                PushIntSixteenToTelemetryVector(W0_Rpm);

                pChrAll->setValue(TelemetryVector);
                pChrAll->notify(true);
            }

            NimBLECharacteristic *pCtrlL = pSvc->getCharacteristic(CTRL_ALL_UUID);
            if (pCtrlL)
            {
                ctrlAllMessage = pCtrlL->getValue();

                if (ctrlAllMessage != "PEE")
                {
                    uint8_t *pData = (uint8_t *) ctrlAllMessage.data();
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


                    ctrlValue0 = (pData[1] << 8) | pData[0] - safetyOffset;
                    ctrlValue1 = (pData[3] << 8) | (pData[2]) - safetyOffset;
                    ctrlValue2 = (pData[5] << 8) | pData[4] - safetyOffset;
                    ctrlValue3 = (pData[7] << 8) | (pData[6]) - safetyOffset;

                    SafeSerialPrintLn(
                            "ctrl: " + String(ctrlValue0) + " " + String(ctrlValue1) + " " + String(ctrlValue2) + " " +
                            String(ctrlValue3));
                }

            }
        }
    }
    else
    {
        bluetoothClientExists = false;
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
            return false;
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
