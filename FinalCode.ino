#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#include "HX711.h"
#include "SparkFun_VL53L1X.h"
#include <Wire.h>
#include <LiquidCrystal.h>  // <-- LCD

// LCD RS, EN, D4, D5, D6, D7
LiquidCrystal lcd(23, 5, 4, 2, 27, 0);

// BLE UUIDs
#define SERVICE_UUID        "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define CHARACTERISTIC_TX   "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
#define CHARACTERISTIC_RX   "6e400002-b5a3-f393-e0a9-e50e24dcca9e"

BLEServer* pServer = nullptr;
BLECharacteristic* pTxCharacteristic;
bool deviceConnected = false;

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
  }
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
  }
};

// VL53L1X
SFEVL53L1X distanceSensor;
float vl53LastValidCm = -1;

// HX711
HX711 scale;
#define DT 32
#define SCK 33

// MUX
const int rowS[4] = {16, 17, 18, 19};
const int rowSIG_L = 25;
const int rowSIG_R = 26;
const int colS[4] = {12, 13, 14, 15};
const int colSIG = 34;

const int rows = 15;
const int cols = 7;
const int colOffsetLeft = 0;
const int colOffsetRight = 7;
const int samples = 10;

// === Nieuwste baseline (geen gewicht) ===
int baselineLeft[15][7] = {
  {1212, 1035, 963, 954, 974, 885, 806},
  {2278, 2201, 2253, 2298, 2241, 2163, 2144},
  {2092, 1955, 1875, 1842, 1839, 1732, 1576},
  {975, 796, 719, 687, 684, 617, 569},
  {1901, 1926, 1969, 1996, 2004, 1946, 1875},
  {846, 817, 801, 858, 914, 790, 733},
  {2587, 2602, 2599, 2531, 2466, 2211, 2020},
  {1486, 1336, 1315, 1327, 1248, 1109, 1009},
  {1908, 1987, 1970, 1922, 1869, 1650, 1519},
  {1733, 1673, 1671, 1697, 1881, 1937, 1744},
  {1276, 1105, 1045, 1044, 1109, 1165, 1147},
  {2433, 2371, 2292, 2269, 2333, 2221, 2092},
  {1840, 1659, 1483, 1411, 1392, 1274, 1169},
  {1966, 1882, 1760, 1761, 1840, 1615, 1469},
  {1191, 1174, 1181, 1184, 1206, 1220, 1391}
};

int baselineRight[15][7] = {
  {792, 690, 667, 659, 683, 675, 624},
  {1296, 1299, 1306, 1321, 1392, 1715, 1843},
  {1804, 1800, 1791, 1840, 1864, 1978, 1762},
  {1136, 1139, 1161, 1212, 1324, 1284, 1129},
  {297, 226, 207, 197, 196, 181, 149},
  {2290, 2310, 2337, 2375, 2382, 2296, 2040},
  {1694, 1739, 1861, 1853, 1754, 1691, 1501},
  {2881, 2863, 2865, 2806, 2738, 2670, 2393},
  {2736, 2744, 2769, 2681, 2627, 2479, 2231},
  {2963, 2951, 2943, 2892, 2852, 2738, 2459},
  {1292, 1385, 1308, 1264, 1286, 1334, 1295},
  {2704, 2728, 2716, 2638, 2629, 2480, 2224},
  {2548, 2556, 2559, 2623, 2673, 2417, 2161},
  {1492, 1267, 1186, 1156, 1143, 1090, 970},
  {718, 608, 586, 588, 584, 581, 568}
};

// === Gekalibreerde maxwaarden bij ±105kg belasting ===
int maxRangeLeft[15][7] = {
  {3388, 3391, 3405, 3394, 3423, 3391, 3405},
  {3537, 3568, 3580, 3568, 3619, 3554, 3572},
  {3490, 3538, 3548, 3544, 3567, 3536, 3536},
  {3479, 3534, 3530, 3522, 3563, 3503, 3508},
  {3448, 3499, 3511, 3520, 3541, 3485, 3471},
  {3513, 3547, 3538, 3544, 3569, 3513, 3506},
  {3508, 3546, 3576, 3566, 3615, 3555, 3566},
  {3495, 3521, 3524, 3528, 3565, 3516, 3543},
  {3519, 3563, 3570, 3565, 3610, 3532, 3532},
  {3491, 3529, 3540, 3533, 3569, 3497, 3471},
  {3550, 3575, 3603, 3583, 3632, 3548, 3532},
  {3558, 3574, 3590, 3589, 3624, 3541, 3529},
  {3471, 3487, 3492, 3498, 3529, 3450, 3429},
  {3427, 3441, 3465, 3460, 3489, 3435, 3426},
  {3309, 3304, 3319, 3312, 3352, 3280, 3279}
};

int maxRangeRight[15][7] = {
  {3051, 3015, 3002, 2997, 2994, 2969, 2972},
  {3261, 3211, 3210, 3187, 3184, 3161, 3149},
  {3387, 3404, 3432, 3412, 3388, 3354, 3353},
  {3444, 3454, 3477, 3446, 3447, 3408, 3402},
  {3369, 3403, 3436, 3416, 3423, 3370, 3367},
  {3208, 3229, 3267, 3264, 3290, 3249, 3253},
  {3274, 3326, 3343, 3339, 3364, 3339, 3323},
  {3347, 3384, 3409, 3426, 3425, 3397, 3383},
  {3405, 3438, 3476, 3482, 3483, 3441, 3447},
  {3423, 3454, 3487, 3494, 3485, 3451, 3468},
  {3432, 3477, 3523, 3519, 3492, 3470, 3488},
  {3518, 3570, 3597, 3604, 3582, 3542, 3548},
  {3532, 3584, 3603, 3589, 3589, 3532, 3538},
  {3423, 3448, 3486, 3473, 3470, 3440, 3440},
  {3012, 3008, 3022, 3032, 3074, 3045, 3013}
};

// Normalisatie
int normalize(int raw, int base, int maxVal) {
  int range = maxVal - base;
  if (range <= 0) return 0;
  float norm = (float)(raw - base) / range;
  norm = constrain(norm, 0.0f, 1.0f);
  float expo = 2.5;
  norm = pow(norm, expo);
  return (int)(norm * 1024);
}

void setMuxChannel(const int* pins, int channel) {
  for (int i = 0; i < 4; i++) {
    digitalWrite(pins[i], (channel >> i) & 0x01);
  }
}

int smoothAnalog(int pin, int count) {
  long total = 0;
  for (int i = 0; i < count; i++) {
    total += analogRead(pin);
    delayMicroseconds(300);
  }
  return total / count;
}

void setup() {
  Wire.begin(21, 22);
  Serial.begin(115200);

  // LCD
  lcd.begin(16, 2);
  lcd.print("Start Drukmat...");

  // BLE
  BLEDevice::init("ESP32-BLE-Drukmat");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);
  pTxCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_TX,
    BLECharacteristic::PROPERTY_NOTIFY);
  pTxCharacteristic->addDescriptor(new BLE2902());

  pService->createCharacteristic(
    CHARACTERISTIC_RX,
    BLECharacteristic::PROPERTY_WRITE);

  pService->start();
  pServer->getAdvertising()->start();

  // HX711
  scale.begin(DT, SCK);
  scale.set_scale(23750);
  scale.tare();

  // VL53L1X
  distanceSensor.begin();
  distanceSensor.setDistanceModeLong();
  distanceSensor.setTimingBudgetInMs(100);
  distanceSensor.startRanging();

  // MUX
  for (int i = 0; i < 4; i++) {
    pinMode(rowS[i], OUTPUT);
    pinMode(colS[i], OUTPUT);
  }
  pinMode(rowSIG_L, OUTPUT);
  pinMode(rowSIG_R, OUTPUT);
  digitalWrite(rowSIG_L, LOW);
  digitalWrite(rowSIG_R, LOW);
}

void loop() {
  if (!deviceConnected) return;

  // Meet gewicht en lengte
  float gewicht = scale.get_units(5);
  if (distanceSensor.checkForDataReady()) {
    int d = distanceSensor.getDistance();
    distanceSensor.clearInterrupt();
    if (d > 0) vl53LastValidCm = (0.969 * d + 2.0) / 10.0;
  }

  // LCD-update
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Gewicht:");
  lcd.print(gewicht, 1);
  lcd.print("kg");

  lcd.setCursor(0, 1);
  lcd.print("Lengte:");
  if (vl53LastValidCm > 0) {
    lcd.print(vl53LastValidCm, 1);
    lcd.print("cm");
  } else {
    lcd.print("Onbekend");
  }

  // BLE verzenden
  pTxCharacteristic->setValue("---START---");
  pTxCharacteristic->notify();
  delay(5);

  String gewichtStr = "WEIGHT:" + String(gewicht, 1);
  pTxCharacteristic->setValue(gewichtStr.c_str());
  pTxCharacteristic->notify();
  delay(5);

  String lengteStr = "HEIGHT:";
  lengteStr += (vl53LastValidCm > 0 ? String(vl53LastValidCm, 1) : "NaN");
  pTxCharacteristic->setValue(lengteStr.c_str());
  pTxCharacteristic->notify();
  delay(5);

  // Linkervoet
  for (int r = 0; r < rows; r++) {
    String rowStr = "";
    for (int c = 0; c < cols; c++) {
      setMuxChannel(rowS, r);
      digitalWrite(rowSIG_L, HIGH);
      setMuxChannel(colS, c + colOffsetLeft);
      delayMicroseconds(100);
      int raw = smoothAnalog(colSIG, samples);
      digitalWrite(rowSIG_L, LOW);
      int norm = normalize(raw, baselineLeft[r][c], maxRangeLeft[r][c]);
      rowStr += String(norm);
      if (c < cols - 1) rowStr += ",";
    }
    pTxCharacteristic->setValue(rowStr.c_str());
    pTxCharacteristic->notify();
    delay(5);
  }

  // Rechtervoet
  for (int r = 0; r < rows; r++) {
    String rowStr = "";
    for (int c = 0; c < cols; c++) {
      setMuxChannel(rowS, r);
      digitalWrite(rowSIG_R, HIGH);
      setMuxChannel(colS, c + colOffsetRight);
      delayMicroseconds(100);
      int raw = smoothAnalog(colSIG, samples);
      digitalWrite(rowSIG_R, LOW);
      int norm = normalize(raw, baselineRight[r][c], maxRangeRight[r][c]);
      rowStr += String(norm);
      if (c < cols - 1) rowStr += ",";
    }
    pTxCharacteristic->setValue(rowStr.c_str());
    pTxCharacteristic->notify();
    delay(5);
  }

  delay(100);  // Frame limiter
}
