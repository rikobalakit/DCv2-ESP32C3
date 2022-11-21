#include <main.h>

// lifecycle

void setup()
{
    Serial.begin(115200);
    Wire.begin();
    SetupDisplay();
    SetupBLE();
    SetupIMU();
    SetupAccelerometer();
    SetupVoltageReader();
    SetupMotors();
}

void loop()
{
    GetIMUData();
    GetAccelerometerData();
    GetVoltageData();
    DisplayText(0.01, _line0, _line1, _line2, _line3, _line4, _line5);
    SetDataForBroadcast();
    SetMotorOutputs();
}

// setup

void SetupDisplay()
{
    if (!_displayEnabled)
    {
        return;
    }

    Display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    DisplayText(1, "DCv2 Prototype");

    _line0 = "";
    _line1 = "";
    _line2 = "";
    _line3 = "";
    _line4 = "";
    _line5 = "";

    delay(100);
}

void SetupIMU()
{
    if (!bno.begin())
    {
        DisplayText(1, "IMU Error", "Bad Init");
    }
    else
    {
        DisplayText(0.25, "IMU Success");
    }
}

void SetupAccelerometer()
{
    if (!lis.begin_I2C(0x19))
    {
        DisplayText(1, "Accel Error", "Bad Init");
    }
    else
    {
        DisplayText(0.25, "Accel Success");
    }
}

void SetupBLE()
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
    _testServoL.attach(PIN_TEST_SERVO_L, 1000, 2000);
    _testServoL.setPeriodHertz(100);

    _testServoR.attach(PIN_TEST_SERVO_R, 1000, 2000);
    _testServoR.setPeriodHertz(100);
}

void SetupVoltageReader()
{
    pinMode(PIN_VOLTAGE_READER, INPUT);
}

// inputs

void GetIMUData()
{
    sensors_event_t event;
    bno.getEvent(&event);

    uint8_t system, gyro, accel, mag;
    system = gyro = accel = mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);

    _line0 = "Orientation";


    _line1 =
            "(" + String(event.orientation.x) + "," + String(event.orientation.y) + "," +
            String(event.orientation.z) + ")";
    orientationX = event.orientation.x;
    orientationY = event.orientation.y;
    orientationZ = event.orientation.z;


}

void GetAccelerometerData()
{

}

void GetVoltageData()
{
    voltageReadingRaw = analogRead(PIN_VOLTAGE_READER);
    // batt is 15.38, v after divider is 1.392, raw is
    voltageReadingMv = (short)((float)(voltageReadingRaw) * (7.907455));
}

// outputs

void SetMotorOutputs()
{
    /*
    SetMotorOutput(_testServoL, ctrlValue0);
    SetMotorOutput(_testServoR, ctrlValue1);
     */
    _testServoL.write(ctrlValue0);
    _testServoR.write(ctrlValue1);
}

void SetDataForBroadcast()
{
    _line2 = "-";
    _line3 = String(voltageReadingRaw)+ " raw, " + String((float)voltageReadingMv/(float)1000) + "V";
    _line4 = "ctrl: " + String(ctrlAllMessage) + " "  + String(ctrlValue0) + " " + String(ctrlValue1);


    if (pServer->getConnectedCount())
    {
        NimBLEService *pSvc = pServer->getServiceByUUID(SERVICE_UUID);
        if (pSvc)
        {
            NimBLECharacteristic *pChrAll = pSvc->getCharacteristic(ORIENTATION_ALL_UUID);
            if (pChrAll)
            {
                short xShort = (short)orientationX;
                short yShort = (short)orientationY;
                short zShort = (short)orientationZ;


                byte orientationByte[8];
                orientationByte[0] =  xShort & 0x00ff;
                orientationByte[1] = (xShort & 0xff00) >> 8;
                orientationByte[2] =  yShort & 0x00ff;
                orientationByte[3] = (yShort & 0xff00) >> 8;
                orientationByte[4] =  zShort & 0x00ff;
                orientationByte[5] = (zShort & 0xff00) >> 8;
                orientationByte[6] =  voltageReadingMv & 0x00ff;
                orientationByte[7] = (voltageReadingMv & 0xff00) >> 8;


                pChrAll->setValue(orientationByte);
                pChrAll->notify(true);
                _line2 += "all";
            }

            NimBLECharacteristic *pCtrlL = pSvc->getCharacteristic(CTRL_ALL_UUID);
            if (pCtrlL)
            {
                ctrlAllMessage = pCtrlL->getValue();

                if(ctrlAllMessage != "PEE")
                {
                    int ctrlAsInt = ctrlAllMessage.getValue<int>();

                    byte ctrlByteArray[4];
                    ctrlByteArray[0] =  ctrlAsInt & 0x000000ff;
                    ctrlByteArray[1] = (ctrlAsInt & 0x0000ff00) >> 8;
                    ctrlByteArray[2] =  (ctrlAsInt & 0x00ff0000) >> 16;
                    ctrlByteArray[3] = (ctrlAsInt & 0xff000000) >> 24;


                    ctrlValue0 = (ctrlByteArray[1] << 8) | ctrlByteArray[0];
                    ctrlValue1 = (ctrlByteArray[3] << 8) | (ctrlByteArray[2]);

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
