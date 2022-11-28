#include <main.h>

// lifecycle

void setup()
{
    Serial.begin(115200);
    Serial.println("hi poop");
    Wire.begin();
    SetupDisplay();
    SetupBLE();
    //SetupIMU();
    SetupAccelerometer();
    SetupVoltageReader();
    SetupMotors();
    SetupWeaponTelemetryConnection();
}

void loop()
{
    motorOutputsSetDuringTelemetryReading = false;
    
    //GetIMUData();
    GetAccelerometerData();
    GetVoltageData();
    GetWeaponTelemetry();
    DisplayText(0.01, _line0, _line1, _line2, _line3, _line4, _line5);
    SetDataForBroadcast();
    if(!motorOutputsSetDuringTelemetryReading)
    {
        SetMotorOutputs();
    }
}

// setup

void SetupDisplay()
{
    if (!_displayEnabled)
    {
        return;
    }

    delay(100);

    Display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    DisplayText(1, "DCv2 Prototype");

    _line0 = "";
    _line1 = "";
    _line2 = "";
    _line3 = "";
    _line4 = "";
    _line5 = "";
}

void SetupIMU()
{
    delay(100);

    if (!bno.begin_I2C())
    {
        DisplayText(1, "IMU Error", "Bad Init");
    }
    else
    {
        DisplayText(0.25, "IMU Success");
        bno.enableReport(SH2_GAME_ROTATION_VECTOR);
    }
}

void SetupAccelerometer()
{
    delay(100);

    if (!lis.begin_I2C())
    {
        DisplayText(1, "Accel Error", "Bad Init");
    }
    else
    {
        DisplayText(0.25, "Accel Success");
        lis.setRange(H3LIS331_RANGE_400_G);
    }
}

void SetupBLE()
{
    delay(100);

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
            ORIENTATION_ALL_UUID,
            NIMBLE_PROPERTY::READ |
            NIMBLE_PROPERTY::NOTIFY
    );

    pOrientationAll->setValue("INIT");

    NimBLECharacteristic *pCtrlAll = pService->createCharacteristic(
            CTRL_ALL_UUID,
            NIMBLE_PROPERTY::READ |
            NIMBLE_PROPERTY::WRITE_NR
    );

    pCtrlAll->setValue("PEE");

    pService->start();
    // BLEAdvertising *pAdvertising = pServer->getAdvertising();  // this still is working for backward compatibility
    NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
    pAdvertising->setMinPreferred(0x12);
    pAdvertising->start();

    DisplayText(0.25, "BLE Success", String(BLEName.c_str()));
    _line5 = "BLE Name: DCv2";
}

void SetupMotors()
{
    delay(100);

    DisplayText(0.25, "Attaching motors", "Pins: " + String(PIN_TEST_SERVO_L) + ", " + String(PIN_TEST_SERVO_R));

    ESP32_ISR_Servos.useTimer(USE_ESP32_TIMER_NO);

    servoLIndex = ESP32_ISR_Servos.setupServo(PIN_TEST_SERVO_L, 0, 2000);
    servoRIndex = ESP32_ISR_Servos.setupServo(PIN_TEST_SERVO_R, 0, 2000);
    servoW0Index = ESP32_ISR_Servos.setupServo(PIN_TEST_SERVO_W0, 0, 2000);
    servoW1Index = ESP32_ISR_Servos.setupServo(PIN_TEST_SERVO_W1, 0, 2000);

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


void SetupVoltageReader()
{
    pinMode(PIN_VOLTAGE_READER, INPUT);
}

// inputs

void GetIMUData()
{
    sh2_SensorValue_t sensorValues;
    bno.getSensorEvent(&sensorValues);

    uint8_t system, gyro, accel, mag;
    system = gyro = accel = mag = 0;

    _line0 = "Orientation";


    _line1 =
            "(" + String(sensorValues.un.gameRotationVector.i) + "," + String(sensorValues.un.gameRotationVector.j) +
            "," +
            String(sensorValues.un.gameRotationVector.k) + "," + String(sensorValues.un.gameRotationVector.real) + ")";
    orientationX = sensorValues.un.gameRotationVector.i;
    orientationY = sensorValues.un.gameRotationVector.j;
    orientationZ = sensorValues.un.gameRotationVector.k;


}

void GetAccelerometerData()
{
    sensors_event_t event;
    lis.getEvent(&event);

    _line0 = "Acceleration";
    _line1 =
            "(" + String(event.acceleration.x) + "," + String(event.acceleration.x) +
            "," +
            String(event.acceleration.x) + ")";
}

void GetVoltageData()
{
    voltageReadingRaw = analogRead(PIN_VOLTAGE_READER);
    // batt is 15.38, v after divider is 1.392, raw is
    voltageReadingMv = (short) ((float) (voltageReadingRaw) * (7.907455));
}

void SetupWeaponTelemetryConnection()
{
    Serial0.begin(115200); // open seria0 for serial monitor

    ulong telemetryRequestWidth = 55;

    ESP32_ISR_Servos.setPulseWidth(servoW0Index, telemetryRequestWidth);
}

void GetWeaponTelemetry()
{
    //read in telemetry from serial
    
    ulong microsAtStart = micros();
    
    ulong telemetryRequestWidth = 50;

    ESP32_ISR_Servos.setPulseWidth(servoW0Index, telemetryRequestWidth);
    
    delay(20);
    
    SetMotorOutputs();
    motorOutputsSetDuringTelemetryReading = true;
    
    isReadingTelemetry = true;

    
    
    ulong microsAfterSendSignal = micros();
    
    while ((micros()-microsAfterSendSignal < 20000) && receivedBytes < 10)
    {
        if(Serial0.available())
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

            W0_Temperature = (float)ESC_telemetrie[0];
            W0_Voltage = (float)ESC_telemetrie[1]/100;
            W0_Current = (float)ESC_telemetrie[2]/100;
            W0_UsedMah = (float)ESC_telemetrie[3];
            W0_Rpm = (float)ESC_telemetrie[4]*1000;

            //DisplayText(0.1f, "temp: " + String(ESC_telemetrie[0]), "used mA/h: " + String(ESC_telemetrie[3]));
            _line5 = "T:" + String(ESC_telemetrie[0]) + "C, RPM:"+String(ESC_telemetrie[4]);
        }
    }
    else
    {
        //DisplayText(0.1f, "failed read", "not enough bytes");
        _line5 = "no read, bytes: " + String(receivedBytes);
    }

    ulong microsAfterDecodeResponse = micros();
    
    receivedBytes = 0;
    
    ulong microsAtEnd = micros();

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

void SetMotorOutputs()
{
    /*
    SetMotorOutput(_testServoL, ctrlValue0);
    SetMotorOutput(_testServoR, ctrlValue1);
     */



    if (servoLIndex != -1)
    {
        //ESP32_ISR_Servos.setPosition(servoLIndex, ctrlValue0);
    }
    if (servoRIndex != -1)
    {
        //ESP32_ISR_Servos.setPosition(servoRIndex, ctrlValue1);
    }
    if (servoW0Index != -1)
    {
        ulong neutralPosition = 1500;
        ulong controlPosition = 1000 + ctrlValue2 * (2000 / 180);
        //ESP32_ISR_Servos.setPosition(servoW0Index, ctrlValue2);
        ESP32_ISR_Servos.setPulseWidth(servoW0Index, controlPosition);
    }
    if (servoW1Index != -1)
    {
        //ESP32_ISR_Servos.setPosition(servoW1Index, ctrlValue3);
    }
}

void SetDataForBroadcast()
{
    _line2 = "-";
    _line3 = String(voltageReadingRaw) + " raw, " + String((float) voltageReadingMv / (float) 1000) + "V";
    _line4 = "ctrl: " + String(ctrlValue0) + " " + String(ctrlValue1) + " " + String(ctrlValue2) + " " +
             String(ctrlValue3);


    if (pServer->getConnectedCount())
    {
        NimBLEService *pSvc = pServer->getServiceByUUID(SERVICE_UUID);
        if (pSvc)
        {
            NimBLECharacteristic *pChrAll = pSvc->getCharacteristic(ORIENTATION_ALL_UUID);
            if (pChrAll)
            {
                short xShort = (short) orientationX;
                short yShort = (short) orientationY;
                short zShort = (short) orientationZ;


                byte orientationByte[8];
                orientationByte[0] = xShort & 0x00ff;
                orientationByte[1] = (xShort & 0xff00) >> 8;
                orientationByte[2] = yShort & 0x00ff;
                orientationByte[3] = (yShort & 0xff00) >> 8;
                orientationByte[4] = zShort & 0x00ff;
                orientationByte[5] = (zShort & 0xff00) >> 8;
                orientationByte[6] = voltageReadingMv & 0x00ff;
                orientationByte[7] = (voltageReadingMv & 0xff00) >> 8;


                pChrAll->setValue(orientationByte);
                pChrAll->notify(true);
                _line2 += "all";
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

                }

            }
        }
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

void DisplayText(float timeSeconds, String line0, String line1, String line2, String line3, String line4, String line5)
{
    if (!_displayEnabled)
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
