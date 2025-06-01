#include "globals.h"

//===========HX711==============//
float finalWeight = 0;
//==============================//

//==========VL53L1X=============//
float readings[SAMPLES_PER_CYCLE];
int count = 0;
float sum = 0;
float mean = 0;
float filteredSum = 0;
float filteredCount = 0;
float outlierCount = 0;
float outlierRatio = 0;
float finalHeigth = 0;

void updateHeightSensor()
{
  if (currentState == RESULTS)
  {
    if (count == 0)
    {
      finalHeigth = -1;
      return;
    }

    for (int i = 0; i < count; i++) { sum += readings[i]; }
    mean = sum / count;

    for (int i = 0; i < count; i++)
    {
      if (abs(readings[i] - mean) <= DEVIATION_THRESHOLD_CM)
      {
        filteredSum += readings[i];
        filteredCount++;
      }
      else { outlierCount++; }
    }

    outlierRatio = (float)outlierCount / count;
    if (outlierRatio > 0.2) { return; }

    finalHeigth = filteredSum / filteredCount;
    return;
  }

  if (currentState == APPROVED && timerStarts)
  {
    if (distanceSensor.checkForDataReady() && count < SAMPLES_PER_CYCLE - 1)
    {
      uint16_t distance = distanceSensor.getDistance();
      readings[count++] = distance / 10.0;
      distanceSensor.clearInterrupt();
      return;
    }
  }
}

void resetDistanceVariables()
{
  for (int i = 0; i < 10; i++) { readings[i] = 0.0; }
  count = 0;
  sum = 0;
  mean = 0;
  filteredSum = 0;
  filteredCount = 0;
  outlierCount = 0;
  outlierRatio = 0;
  finalHeigth = 0;
}
//==============================//

//=========VELOSTAT=============//
void setMuxChannel(const int* pins, int channel)
{
  for (int i = 0; i < 4; i++) { digitalWrite(pins[i], (channel >> i) & 0x01); }
}

int getAverageReading(int pin, int count)
{
  long total = 0;
  for (int i = 0; i < count; i++)
  {
    total += analogRead(pin);
    delayMicroseconds(300);
  }
  return total / count;
}

int mapWithCurve(int rawValue, int baseValue, int maxValue)
{
  int range = maxValue - baseValue;
  if (range <= 0) { return 0; }
  float norm = (float)(rawValue - baseValue) / range;
  norm = constrain(norm, 0.0f, 1.0f);
  return (int)(pow(norm, 2.5) * 1024);
}

void readMatrices(uint16_t matL[15][7], uint16_t matR[15][7])
{
  for (int r = 0; r < 15; r++)
  {
    for (int c = 0; c < 7; c++)
    {
      setMuxChannel(rowMux, r);
      digitalWrite(rowSigL, HIGH);
      setMuxChannel(colMux, c);
      delayMicroseconds(100);
      int raw = getAverageReading(colSig, samples);
      digitalWrite(rowSigL, LOW);
      matL[r][c] = mapWithCurve(raw, baselineLeft[r][c], maxRangeLeft[r][c]);

      digitalWrite(rowSigR, HIGH);
      setMuxChannel(colMux, c + 7);
      delayMicroseconds(100);
      raw = getAverageReading(colSig, samples);
      digitalWrite(rowSigR, LOW);
      matR[r][c] = mapWithCurve(raw, baselineRight[r][c], maxRangeRight[r][c]);
    }
  }
}
//==============================//

//============BLE===============//
class RxCallbacks : public BLECharacteristicCallbacks
{
  void onWrite(BLECharacteristic* pChar) override
  {
    String cmd = String(pChar->getValue().c_str());
    processBLECommand(cmd);
  }
};

class MyServerCallbacks : public BLEServerCallbacks
{
  void onConnect(BLEServer* pServer) { deviceConnected = true; }
  void onDisconnect(BLEServer* pServer)
  {
    deviceConnected = false;
    currentState = PAIRING;
    awaitingResponse = false;
    pServer->getAdvertising()->start();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Pressure Mat");
    lcd.setCursor(0, 1);
    lcd.print("Discoverable...");
  }
};

void processBLECommand(String cmd)
{
  if (cmd == "CONNECTED")
  {
    currentState = CONNECTED;
    lcd.clear();
    lcd.print("Connected");
  }
  else if (cmd == "START")
  {
    resetDistanceVariables();
    currentState = START;
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Checking");
    lcd.print(0, 1);
    lcd.print("posture...");
  }
  else if (cmd == "DISAPPROVED")
  {
    resetDistanceVariables();
    awaitingResponse = false;
    currentState = DISAPPROVED;
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Posture is");
    lcd.setCursor(0, 1);
    lcd.print("disapproved");
  }
  else if (cmd == "APPROVED")
  {
    awaitingResponse = false;
    currentState = APPROVED;
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Posture is");
    lcd.setCursor(0, 1);
    lcd.print("approved");
  }
  else if (cmd == "START_TIMER")
  {
    timerStarts = true;
    timerCancel = false;
    timerFinish = false;
  }
  else if (cmd == "CANCEL_TIMER")
  {
    timerStarts = false;
    timerCancel = true;
    timerFinish = false;
    resetDistanceVariables();
  }
  else if (cmd == "FINISH_TIMER")
  {
    timerStarts = false;
    timerCancel = false;
    timerFinish = true;
    currentState = RESULTS;
  }
  else if (cmd == "KILL")
  {
    currentState = PAIRING;
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Pressure Mat");
    lcd.setCursor(0, 1);
    lcd.print("Discoverable...");
  }
}

void sendMatrixPacket(uint8_t footId, uint16_t matrixData[15][7], bool viaBle, bool viaWiFi)
{
  uint8_t packet[212];
  packet[0] = footId;
  int idx = 1;
  for (int r = 0; r < 15; r++)
  {
    for (int c = 0; c < 7; c++)
    {
      int colIndex = (footId == 'R') ? (6 - c) : c;
      uint16_t val = matrixData[r][colIndex];
      packet[idx++] = (val >> 8) & 0xFF;
      packet[idx++] = val & 0xFF;
    }
  }

  if (viaBle && deviceConnected)
  {
    pTxCharacteristic->setValue(packet, sizeof(packet));
    pTxCharacteristic->notify();
  }

  if (viaWiFi && wifiReady)
  {
    udp.beginPacket(udpAddress, udpPort);
    udp.write(packet, sizeof(packet));
    udp.endPacket();
  }
}

void sendResultPacket(float finalWeight, float finalLength)
{
  uint8_t data[10];
  data[0] = 0x57; // W
  memcpy(&data[1], &finalWeight, 4);
  data[5] = 0x4C; // L
  memcpy(&data[6], &finalLength, 4);
  pTxCharacteristic->setValue(data, 10);
  pTxCharacteristic->notify();
}
//==============================//

void setup()
{
  //========VL53L1X=========//
  resetDistanceVariables();
  Wire.begin();
  while (distanceSensor.begin() != 0) { delay(100); }
  distanceSensor.setDistanceModeLong();
  distanceSensor.setTimingBudgetInMs(TIMING_BUDGET_MS);
  distanceSensor.startRanging();
  //========================//

  //==========LCD===========//
  lcd.begin(16, 2);
  lcd.print("Pressure Mat");
  lcd.setCursor(0, 1);
  lcd.print("Discoverable...");
  //========================//

  //==========BLE===========//
  BLEDevice::init("MeasureMates");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService* service = pServer->createService(SERVICE_UUID);
  pTxCharacteristic =
      service->createCharacteristic(CHARACTERISTIC_TX, BLECharacteristic::PROPERTY_NOTIFY);
  pTxCharacteristic->addDescriptor(new BLE2902());
  pRxCharacteristic =
      service->createCharacteristic(CHARACTERISTIC_RX, BLECharacteristic::PROPERTY_WRITE);
  pRxCharacteristic->setCallbacks(new RxCallbacks());
  service->start();
  BLEAdvertisementData advData;
  advData.setName("MeasureMates");
  pServer->getAdvertising()->setAdvertisementData(advData);
  pServer->getAdvertising()->start();
  //========================//

  //==========WIFI==========//
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts++ < 20); { delay(500); }
  if (WiFi.status() == WL_CONNECTED)
  {
    udp.begin(udpPort);
    wifiReady = true;
  }
  else { wifiReady = false; }
  //========================//

  //=========HX711==========//
  scale.begin(DT, SCK);
  scale.set_scale(23750);
  scale.tare();
  //========================//

  for (int i = 0; i < 4; i++)
  {
    pinMode(rowMux[i], OUTPUT);
    pinMode(colMux[i], OUTPUT);
  }
  pinMode(rowSigL, OUTPUT);
  pinMode(rowSigR, OUTPUT);
  digitalWrite(rowSigL, LOW);
  digitalWrite(rowSigR, LOW);
}

void loop()
{
  // Send matrix packets through WiFi 5 times a second
  if (wifiReady && millis() - lastWifiSendTime > 200)
  {
    lastWifiSendTime = millis();
    readMatrices(matrixLwifi, matrixRwifi);
    sendMatrixPacket('L', matrixLwifi, false, true);
    delay(5);
    sendMatrixPacket('R', matrixRwifi, false, true);
  }

  switch (currentState)
  {
    case PAIRING:
    {
      if (deviceConnected)
      {
        currentState = CONNECTED;
        break;
      }
    }
    case CONNECTED:
    {
      break;
    }
    case START:
    {
      awaitingResponse = false;
    }
    case DISAPPROVED:
    {
      if (awaitingResponse) { break; }
      uint16_t matrixL[15][7];
      uint16_t matrixR[15][7];
      readMatrices(matrixL, matrixR);
      sendMatrixPacket('L', matrixL, true, false);
      delay(5);
      sendMatrixPacket('R', matrixR, true, false);
      delay(5);
      awaitingResponse = true;
      finalWeight = -1;
      break;
    }
    case APPROVED:
    {
      updateHeightSensor();
      finalWeight = scale.get_units(5);
      if (awaitingResponse) { break; }
      uint16_t matrixL[15][7];
      uint16_t matrixR[15][7];
      readMatrices(matrixL, matrixR);
      sendMatrixPacket('L', matrixL, true, false);
      delay(5);
      sendMatrixPacket('R', matrixR, true, false);
      delay(5);
      awaitingResponse = true;
      break;
    }
    case RESULTS:
    {
      updateHeightSensor();
      sendResultPacket(finalWeight, finalHeigth);
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Weight: ");
      lcd.print(finalWeight, 1);
      lcd.setCursor(0, 1);
      lcd.print("Length: ");
      lcd.print(finalHeigth, 1);
      awaitingResponse = false;
      break;
    }
    default:
    {
      break;
    }
  }
}
