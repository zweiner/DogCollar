#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <NimBLEDevice.h>
#include <TinyGPSPlus.h>
#include <math.h>
#include <Wire.h>
#include <U8x8lib.h>

// ===================== PINS (ESP32 Thing Plus) =====================
static const int SD_CS = 16;

// GPS
static const int GPS_RX_PIN = 27; // ESP32 RX  <- GPS TX
static const int GPS_TX_PIN = 14; // ESP32 TX  -> GPS RX (optional)

// ADXL335 analog outputs
static const int ACCEL_X_PIN = A0;
static const int ACCEL_Y_PIN = A1;
static const int ACCEL_Z_PIN = A2;

// Button
static const int BUTTON_PIN = 12;

// ===================== SD PATHS =====================
static const char* ROUTES_DIR = "/routes";

// ===================== BLE =====================
static const char* BLE_NAME      = "DogCollar";
static const char* SERVICE_UUID  = "12345678-1234-1234-1234-1234567890ab";
static const char* META_UUID     = "12345678-1234-1234-1234-1234567890ac";
static const char* CTRL_UUID     = "12345678-1234-1234-1234-1234567890ad";
static const char* DATA_UUID     = "12345678-1234-1234-1234-1234567890ae";

// ===================== OLED =====================
U8X8_SSD1306_128X32_UNIVISION_HW_I2C oled(U8X8_PIN_NONE);

const int MAX_REFRESH = 1000;
unsigned long lastClear = 0;

// ===================== SYSTEM MODE =====================
enum SystemMode {
  MODE_ACTIVE,
  MODE_IDLE
};

static SystemMode systemMode = MODE_ACTIVE;

// ===================== GLOBALS =====================
TinyGPSPlus gps;
HardwareSerial gpsSerial(1);
File routeFile;
String currentRoutePath = "";

// BLE
NimBLECharacteristic* metaChar = nullptr;
NimBLECharacteristic* dataChar = nullptr;

// Stream settings
static const uint16_t CHUNK_BYTES = 180;
static bool streaming = false;
static uint32_t fileSize = 0;

// SD / tracking state
static bool sdOk = false;
static bool trackingEnabled = true;   // starts tracking immediately on power-up

// ===================== TIMING =====================
static uint32_t lastDebugMs = 0;
static uint32_t lastAccelSampleMs = 0;
static uint32_t lastCsvLogMs = 0;
static uint32_t lastGpsAcceptMs = 0;
static uint32_t lastMetaUpdateMs = 0;

static const uint32_t ACCEL_SAMPLE_PERIOD_MS = 20;     // 50 Hz internal sampling
static const uint32_t CSV_LOG_PERIOD_MS      = 100;    // 10 Hz CSV append
static const uint32_t GPS_REFRESH_MS         = 10000;  // accept GPS at least every 10 sec
static const uint32_t META_UPDATE_MS         = 1000;   // update BLE metadata once/sec
static const uint32_t DEBUG_PERIOD_MS        = 2000;   // serial debug every 2 sec

// ===================== GPS SENSITIVITY =====================
static const double GPS_MOVE_THRESHOLD_M = 3.0; // sensitive to small movement

static double acceptedLat = 0.0;
static double acceptedLon = 0.0;
static bool haveAcceptedGps = false;

static double lastMoveDistM = 0.0;

// ===================== ACCEL / STEP DETECTION =====================
struct AccelSample {
  int xRaw;
  int yRaw;
  int zRaw;
  float mag;
  float baseline;
  float dyn;
};

static AccelSample latestAccel = {0, 0, 0, 0, 0, 0};

static float magBaseline = 0.0f;
static bool baselineInitialized = false;

// Calmer locomotion behavior
static bool locomotionActive = false;
static uint32_t lastMotionMs = 0;
static uint32_t locomotionStartMs = 0;

static const float MOTION_START_THRESHOLD = 160.0f;
static const float MOTION_STOP_THRESHOLD  = 95.0f;
static const uint32_t MOTION_HOLD_MS      = 1600;

// Calmer step detection
static float prevDyn2 = 0.0f;
static float prevDyn1 = 0.0f;
static uint32_t lastStepMs = 0;
static uint32_t stepCount = 0;

static const float STEP_PEAK_THRESHOLD        = 70.0f;
static const uint32_t STEP_MIN_INTERVAL_MS    = 320;
static const uint32_t STEP_MAX_INTERVAL_MS    = 1500;
static const uint32_t STEP_ENABLE_AFTER_LOCO_MS = 500;
// ===================== CSV CACHE =====================
static bool gpsAcceptedThisCycle = false;

// ===================== BLE META PACKET =====================
#pragma pack(push, 1)
struct MetaPacket {
  uint32_t file_size;
  uint16_t chunk_bytes;
};
#pragma pack(pop)

void setMeta(uint32_t sizeBytes) {
  MetaPacket m{ sizeBytes, CHUNK_BYTES };
  if (metaChar) metaChar->setValue((uint8_t*)&m, sizeof(m));
}

// ---------- OLED helpers ----------
void setupDisplay() {
  oled.begin();
  oled.setPowerSave(0);
  oled.setFont(u8x8_font_amstrad_cpc_extended_r);
  oled.setCursor(0, 0);
}

void writeDisplay(const char * message, int row, bool erase) {
  unsigned long now = millis();
  if (erase && (millis() - lastClear >= MAX_REFRESH)) {
    oled.clearDisplay();
    lastClear = now;
  }
  oled.setCursor(0, row);
  oled.print(message);
}

void showIdleScreen() {
  oled.setPowerSave(0);
  writeDisplay("DEVICE IDLE", 0, true);
  writeDisplay("Press button", 1, false);
  writeDisplay("to wake", 2, false);
  writeDisplay("", 3, false);
  delay(600);
  oled.setPowerSave(1);
}

void showWakeScreen() {
  oled.setPowerSave(0);
  writeDisplay("WAKING UP", 0, true);
  writeDisplay("Tracking ON", 1, false);
  writeDisplay("", 2, false);
  writeDisplay("", 3, false);
}

void updateDisplay() {
  if (systemMode == MODE_IDLE) return;

  static uint32_t lastDisplayMs = 0;
  uint32_t now = millis();
  if (now - lastDisplayMs < 500) return;
  lastDisplayMs = now;

  char row0[20];
  char row1[20];
  char row2[20];
  char row3[20];

  if (gps.location.isValid()) {
    snprintf(row0, sizeof(row0), "GPS: LOCK");
  } else {
    snprintf(row0, sizeof(row0), "GPS: SEARCH");
  }

  if (gps.satellites.isValid() && gps.hdop.isValid()) {
    snprintf(row1, sizeof(row1), "SAT:%lu HD:%.1f",
             (unsigned long)gps.satellites.value(),
             gps.hdop.hdop());
  } else {
    snprintf(row1, sizeof(row1), "SAT:N/A HD:N/A");
  }

  snprintf(row2, sizeof(row2), "STEPS:%lu", (unsigned long)stepCount);

  snprintf(row3, sizeof(row3), "TRK:%s LOC:%s",
           trackingEnabled ? "ON" : "OFF",
           locomotionActive ? "Y" : "N");

  writeDisplay(row0, 0, true);
  writeDisplay(row1, 1, false);
  writeDisplay(row2, 2, false);
  writeDisplay(row3, 3, false);
}

// ---------- Mode control ----------
void enterIdleMode() {
  systemMode = MODE_IDLE;
  trackingEnabled = false;
  locomotionActive = false;

  Serial.println("Entering IDLE mode");
  showIdleScreen();
}

void enterActiveMode() {
  systemMode = MODE_ACTIVE;
  trackingEnabled = true;

  Serial.println("Entering ACTIVE mode");
  showWakeScreen();
}

// ---------- SD helpers ----------
bool sdInit() {
  SPI.begin();
  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, HIGH);
  return SD.begin(SD_CS);
}

bool ensureRoutesDir() {
  if (SD.exists(ROUTES_DIR)) return true;
  return SD.mkdir(ROUTES_DIR);
}

String makeNextRoutePath() {
  if (!ensureRoutesDir()) return "";

  for (int i = 0; i < 10000; i++) {
    char path[40];
    snprintf(path, sizeof(path), "/routes/route_%04d.csv", i);
    if (!SD.exists(path)) {
      return String(path);
    }
  }
  return "";
}

bool createNewRouteFile() {
  currentRoutePath = makeNextRoutePath();
  if (currentRoutePath.length() == 0) return false;

  File f = SD.open(currentRoutePath.c_str(), FILE_WRITE);
  if (!f) return false;

  f.println("lat,lon,t_ms,ax,ay,az,mag,dyn,locomotion,steps,gps_fresh,gps_move_m");
  f.close();

  Serial.print("Created new route file: ");
  Serial.println(currentRoutePath);
  return true;
}

void appendSessionRow(double lat, double lon, uint32_t t_ms,
                      int ax, int ay, int az,
                      float mag, float dyn,
                      bool locomotion, uint32_t steps,
                      bool gpsFresh, double gpsMoveM) {
  if (currentRoutePath.length() == 0) return;

  File f = SD.open(currentRoutePath.c_str(), FILE_APPEND);
  if (!f) {
    Serial.println("appendSessionRow: failed to open active route file");
    return;
  }

  f.print(lat, 6);
  f.print(",");
  f.print(lon, 6);
  f.print(",");
  f.print(t_ms);
  f.print(",");
  f.print(ax);
  f.print(",");
  f.print(ay);
  f.print(",");
  f.print(az);
  f.print(",");
  f.print(mag, 2);
  f.print(",");
  f.print(dyn, 2);
  f.print(",");
  f.print(locomotion ? 1 : 0);
  f.print(",");
  f.print(steps);
  f.print(",");
  f.print(gpsFresh ? 1 : 0);
  f.print(",");
  f.println(gpsMoveM, 2);

  f.close();
}

uint32_t getRouteSize() {
  if (currentRoutePath.length() == 0) return 0;

  File f = SD.open(currentRoutePath.c_str(), FILE_READ);
  if (!f) return 0;

  uint32_t sz = (uint32_t)f.size();
  f.close();
  return sz;
}

void dumpRouteFileToSerial() {
  if (currentRoutePath.length() == 0) {
    Serial.println("No active route file");
    return;
  }

  File f = SD.open(currentRoutePath.c_str(), FILE_READ);
  if (!f) {
    Serial.println("Could not open active route file for reading");
    return;
  }

  Serial.println("==== BEGIN ACTIVE ROUTE FILE ====");
  Serial.print("Path: ");
  Serial.println(currentRoutePath);
  while (f.available()) {
    Serial.write(f.read());
  }
  Serial.println("\n==== END ACTIVE ROUTE FILE ====");
  f.close();
}

void flushFinalSessionRow() {
  if (!sdOk) return;
  if (currentRoutePath.length() == 0) return;

  double lat = 0.0;
  double lon = 0.0;

  if (haveAcceptedGps) {
    lat = acceptedLat;
    lon = acceptedLon;
  } else if (gps.location.isValid()) {
    lat = gps.location.lat();
    lon = gps.location.lng();
  }

  uint32_t now = millis();

  appendSessionRow(
    lat, lon, now,
    latestAccel.xRaw, latestAccel.yRaw, latestAccel.zRaw,
    latestAccel.mag, latestAccel.dyn,
    locomotionActive, stepCount,
    gpsAcceptedThisCycle, lastMoveDistM
  );

  setMeta(getRouteSize());

  Serial.println("Final session row flushed to CSV.");
  Serial.print("Final persisted stepCount = ");
  Serial.println(stepCount);
}

// ---------- BLE file streaming ----------
bool openRouteForStream() {
  if (currentRoutePath.length() == 0) return false;
  if (!SD.exists(currentRoutePath.c_str())) return false;

  routeFile = SD.open(currentRoutePath.c_str(), FILE_READ);
  if (!routeFile) return false;

  fileSize = (uint32_t)routeFile.size();
  setMeta(fileSize);

  Serial.print("Streaming file: ");
  Serial.println(currentRoutePath);

  return true;
}

void closeRouteStream() {
  if (routeFile) routeFile.close();
}

class CtrlCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* c, NimBLEConnInfo& connInfo) override {
    (void)connInfo;
    std::string v = c->getValue();

    if (v == "START") {
      Serial.println("CTRL: START STREAM");

      streaming = false;
      closeRouteStream();

      if (!sdOk) {
        Serial.println("SD not OK - cannot stream.");
        return;
      }

      if (!openRouteForStream()) {
        Serial.println("Failed to open active route file for streaming.");
        return;
      }

      streaming = true;
    }
    else if (v == "STOP") {
      Serial.println("CTRL: STOP STREAM");
      streaming = false;
      closeRouteStream();
    }
    else if (v == "STOP_TRACK") {
      Serial.println("CTRL: STOP TRACKING");
      flushFinalSessionRow();
      trackingEnabled = false;
      locomotionActive = false;
      systemMode = MODE_IDLE;
      showIdleScreen();
    }
    else if (v == "START_TRACK") {
      Serial.println("CTRL: START TRACKING");
      enterActiveMode();
    }
    else {
      Serial.print("CTRL: unknown command: ");
      Serial.println(v.c_str());
    }
  }
};

void setupBLE() {
  NimBLEDevice::init(BLE_NAME);
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);

  NimBLEServer* server = NimBLEDevice::createServer();
  NimBLEService* svc = server->createService(SERVICE_UUID);

  metaChar = svc->createCharacteristic(META_UUID, NIMBLE_PROPERTY::READ);

  NimBLECharacteristic* ctrlChar =
      svc->createCharacteristic(CTRL_UUID, NIMBLE_PROPERTY::WRITE);
  ctrlChar->setCallbacks(new CtrlCallbacks());

  dataChar = svc->createCharacteristic(DATA_UUID, NIMBLE_PROPERTY::NOTIFY);

  if (sdOk && currentRoutePath.length() > 0 && SD.exists(currentRoutePath.c_str())) {
    setMeta(getRouteSize());
  } else {
    setMeta(0);
  }

  svc->start();

  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  adv->addServiceUUID(SERVICE_UUID);
  adv->start();

  Serial.println("BLE advertising started (DogCollar).");
}

// ---------- Accelerometer ----------
void setupAccelerometer() {
  analogReadResolution(12);
  analogSetPinAttenuation(ACCEL_X_PIN, ADC_11db);
  analogSetPinAttenuation(ACCEL_Y_PIN, ADC_11db);
  analogSetPinAttenuation(ACCEL_Z_PIN, ADC_11db);
  Serial.println("Accelerometer init complete");
}

AccelSample readAccelerometer() {
  AccelSample s;

  s.xRaw = analogRead(ACCEL_X_PIN);
  s.yRaw = analogRead(ACCEL_Y_PIN);
  s.zRaw = analogRead(ACCEL_Z_PIN);

  float x = (float)s.xRaw;
  float y = (float)s.yRaw;
  float z = (float)s.zRaw;

  s.mag = sqrtf(x * x + y * y + z * z);

  if (!baselineInitialized) {
    magBaseline = s.mag;
    baselineInitialized = true;
  }

 magBaseline = 0.992f * magBaseline + 0.008f * s.mag;

  s.baseline = magBaseline;
  s.dyn = fabsf(s.mag - s.baseline);

  return s;
}

void updateLocomotionAndSteps(const AccelSample& s, uint32_t now) {
  if (s.dyn > MOTION_START_THRESHOLD) {
    if (!locomotionActive) {
      locomotionStartMs = now;
    }
    locomotionActive = true;
    lastMotionMs = now;
  } else if (locomotionActive && s.dyn > MOTION_STOP_THRESHOLD) {
    lastMotionMs = now;
  } else if (locomotionActive && (now - lastMotionMs > MOTION_HOLD_MS)) {
    locomotionActive = false;
  }

  bool isPeak = (prevDyn1 > prevDyn2) &&
                (prevDyn1 > s.dyn) &&
                ((prevDyn1 - prevDyn2) > 8.0f) &&
                ((prevDyn1 - s.dyn) > 8.0f);

  bool locomotionSettled =
      locomotionActive &&
      ((now - locomotionStartMs) >= STEP_ENABLE_AFTER_LOCO_MS);

  if (locomotionSettled && isPeak && prevDyn1 > STEP_PEAK_THRESHOLD) {
    uint32_t dt = now - lastStepMs;

    if (dt >= STEP_MIN_INTERVAL_MS &&
        (lastStepMs == 0 || dt <= STEP_MAX_INTERVAL_MS)) {
      stepCount++;
      lastStepMs = now;

      Serial.println("******** STEP DETECTED ********");
      Serial.print("count = ");
      Serial.println(stepCount);
      Serial.print("peak dyn = ");
      Serial.println(prevDyn1, 2);
      Serial.print("dt ms = ");
      Serial.println(dt);
      Serial.print("locomotion settled = ");
      Serial.println(locomotionSettled ? "YES" : "NO");
      Serial.println("******************************");
    }
  }

  prevDyn2 = prevDyn1;
  prevDyn1 = s.dyn;
}

void handleAccelerometer() {
  if (systemMode == MODE_IDLE) return;

  uint32_t now = millis();
  if (now - lastAccelSampleMs < ACCEL_SAMPLE_PERIOD_MS) return;
  lastAccelSampleMs = now;

  latestAccel = readAccelerometer();
  updateLocomotionAndSteps(latestAccel, now);
}

// ---------- Button ----------
void handleButton() {
  static bool lastState = HIGH;
  static uint32_t lastPressTime = 0;

  bool currentState = digitalRead(BUTTON_PIN);

  if (currentState == LOW && lastState == HIGH) {
    if (millis() - lastPressTime > 300) {
      if (systemMode == MODE_ACTIVE) {
        enterIdleMode();
      } else {
        enterActiveMode();
      }
      lastPressTime = millis();
    }
  }

  lastState = currentState;
}

// ---------- GPS ----------
void handleGpsSerial() {
  if (systemMode == MODE_IDLE) return;

  while (gpsSerial.available()) {
    char c = (char)gpsSerial.read();
    gps.encode(c);
  }
}

void tryAcceptGpsPoint() {
  gpsAcceptedThisCycle = false;

  if (systemMode == MODE_IDLE) return;
  if (!gps.location.isValid()) return;

  double lat = gps.location.lat();
  double lon = gps.location.lng();
  uint32_t now = millis();

  if (!haveAcceptedGps) {
    acceptedLat = lat;
    acceptedLon = lon;
    haveAcceptedGps = true;
    lastGpsAcceptMs = now;
    lastMoveDistM = 0.0;
    gpsAcceptedThisCycle = true;

    Serial.println("GPS ACCEPTED: first valid fix");
    return;
  }

  double distM = TinyGPSPlus::distanceBetween(
    acceptedLat, acceptedLon,
    lat, lon
  );

  lastMoveDistM = distM;

  bool movedEnough = (distM >= GPS_MOVE_THRESHOLD_M);
  bool refreshDue = (now - lastGpsAcceptMs >= GPS_REFRESH_MS);

  if (movedEnough || refreshDue) {
    acceptedLat = lat;
    acceptedLon = lon;
    lastGpsAcceptMs = now;
    gpsAcceptedThisCycle = true;

    Serial.println("GPS ACCEPTED:");
    Serial.print("  movedEnough = ");
    Serial.println(movedEnough ? "YES" : "NO");
    Serial.print("  refreshDue  = ");
    Serial.println(refreshDue ? "YES" : "NO");
    Serial.print("  moveDistM   = ");
    Serial.println(distM, 2);
  }
}

void printDebugStatus() {
  uint32_t now = millis();
  if (now - lastDebugMs < DEBUG_PERIOD_MS) return;
  lastDebugMs = now;

  Serial.println("---- SYSTEM STATUS ----");

  Serial.print("Mode: ");
  Serial.println(systemMode == MODE_ACTIVE ? "ACTIVE" : "IDLE");

  Serial.print("charsProcessed: ");
  Serial.println(gps.charsProcessed());

  Serial.print("sentencesWithFix: ");
  Serial.println(gps.sentencesWithFix());

  Serial.print("failedChecksum: ");
  Serial.println(gps.failedChecksum());

  Serial.print("location valid: ");
  Serial.println(gps.location.isValid() ? "YES" : "NO");

  Serial.print("location updated: ");
  Serial.println(gps.location.isUpdated() ? "YES" : "NO");

  if (gps.location.isValid()) {
    Serial.print("GPS current LAT: ");
    Serial.println(gps.location.lat(), 6);
    Serial.print("GPS current LON: ");
    Serial.println(gps.location.lng(), 6);
  }

  if (haveAcceptedGps) {
    Serial.print("GPS accepted LAT: ");
    Serial.println(acceptedLat, 6);
    Serial.print("GPS accepted LON: ");
    Serial.println(acceptedLon, 6);
  } else {
    Serial.println("GPS accepted point: NONE");
  }

  Serial.print("GPS move from accepted (m): ");
  Serial.println(lastMoveDistM, 2);

  Serial.print("satellites: ");
  if (gps.satellites.isValid()) Serial.println(gps.satellites.value());
  else Serial.println("N/A");

  Serial.print("hdop: ");
  if (gps.hdop.isValid()) Serial.println(gps.hdop.hdop());
  else Serial.println("N/A");

  Serial.print("Current route file: ");
  if (currentRoutePath.length() > 0) Serial.println(currentRoutePath);
  else Serial.println("(none)");

  Serial.print("Current route size: ");
  Serial.println(getRouteSize());

  Serial.print("Tracking enabled: ");
  Serial.println(trackingEnabled ? "YES" : "NO");

  Serial.print("ACC X:");
  Serial.print(latestAccel.xRaw);
  Serial.print(" Y:");
  Serial.print(latestAccel.yRaw);
  Serial.print(" Z:");
  Serial.println(latestAccel.zRaw);

  Serial.print("MAG: ");
  Serial.print(latestAccel.mag, 1);
  Serial.print("  BASE: ");
  Serial.print(latestAccel.baseline, 1);
  Serial.print("  DYN: ");
  Serial.println(latestAccel.dyn, 1);

  Serial.print("Locomotion active: ");
  Serial.println(locomotionActive ? "YES" : "NO");

  Serial.print("Step count: ");
  Serial.println(stepCount);

  if (!gps.location.isValid() && systemMode == MODE_ACTIVE) {
    Serial.println("Waiting for GPS fix...");
  }

  Serial.println("-----------------------");
}

// ---------- CSV logging ----------
void handleCsvLogging() {
  if (systemMode == MODE_IDLE) return;
  if (streaming) return;
  if (!sdOk) return;
  if (!trackingEnabled) return;
  if (currentRoutePath.length() == 0) return;

  uint32_t now = millis();
  if (now - lastCsvLogMs < CSV_LOG_PERIOD_MS) return;
  lastCsvLogMs = now;

  double lat = 0.0;
  double lon = 0.0;

  if (haveAcceptedGps) {
    lat = acceptedLat;
    lon = acceptedLon;
  } else if (gps.location.isValid()) {
    lat = gps.location.lat();
    lon = gps.location.lng();
  }

  appendSessionRow(
    lat, lon, now,
    latestAccel.xRaw, latestAccel.yRaw, latestAccel.zRaw,
    latestAccel.mag, latestAccel.dyn,
    locomotionActive, stepCount,
    gpsAcceptedThisCycle, lastMoveDistM
  );
}

void handleMetaUpdate() {
  if (!sdOk) return;
  uint32_t now = millis();
  if (now - lastMetaUpdateMs < META_UPDATE_MS) return;
  lastMetaUpdateMs = now;
  setMeta(getRouteSize());
}

// ---------- Setup / loop ----------
void setup() {
  Serial.begin(115200);
  delay(400);
  Serial.println("\n=== Route Sync Firmware (Thing Plus: GPS + ADXL335 + SD + BLE + OLED + Soft Idle) ===");

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  Wire.begin();
  setupDisplay();
  writeDisplay("Booting...", 0, true);
  writeDisplay("Init SD...", 1, false);

  Serial.println("Init SD...");
  sdOk = sdInit();
  if (!sdOk) {
    Serial.println("SD init failed");
    writeDisplay("SD FAIL", 1, false);
  } else {
    Serial.println("SD init OK");
    writeDisplay("SD OK", 1, false);

    if (!ensureRoutesDir()) {
      Serial.println("Failed to ensure /routes directory");
      writeDisplay("DIR FAIL", 2, false);
    } else {
      Serial.println("/routes directory ready");
      writeDisplay("DIR OK", 2, false);
    }

    if (!createNewRouteFile()) {
      Serial.println("Failed to create new session route file");
      writeDisplay("FILE FAIL", 3, false);
    } else {
      Serial.print("Active session file: ");
      Serial.println(currentRoutePath);
      dumpRouteFileToSerial();
      writeDisplay("FILE OK", 3, false);
    }
  }

  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.println("GPS serial started @9600 on RX=27 TX=14");

  setupAccelerometer();
  setupBLE();

  Serial.println("Tracking starts automatically on power-up.");
  Serial.println("Button toggles ACTIVE <-> IDLE.");
}

void loop() {
  handleButton();

  if (systemMode == MODE_ACTIVE) {
    handleGpsSerial();
    handleAccelerometer();
    tryAcceptGpsPoint();
    handleCsvLogging();
    updateDisplay();
  }

  handleMetaUpdate();
  printDebugStatus();

  if (streaming && routeFile) {
    uint8_t buf[CHUNK_BYTES];
    int n = routeFile.read(buf, CHUNK_BYTES);

    if (n > 0) {
      dataChar->setValue(buf, n);
      dataChar->notify();
      delay(8);
    } else {
      Serial.println("Stream complete.");
      streaming = false;
      closeRouteStream();
    }
  }

  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == 'D' || cmd == 'd') {
      dumpRouteFileToSerial();
    }
  }

  if (systemMode == MODE_IDLE) {
    delay(50);
  } else {
    delay(2);
  }
}
