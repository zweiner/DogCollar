#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <NimBLEDevice.h>
#include <TinyGPSPlus.h>

// ===================== PINS (ESP32 Thing Plus) =====================
// SD (SPI) pins: use default VSPI pins explicitly
// Most ESP32 boards use: SCK=18, MISO=19, MOSI=23
// CS we choose as GPIO15 (you have this pin broken out)

static const int SD_CS   = 17;

// GPS pins (you said you have 27 and 14 available)
// GPS TX -> ESP RX (GPIO27)
// GPS RX -> ESP TX (GPIO14)
static const int GPS_RX_PIN = 27; // ESP32 RX  <- GPS TX
static const int GPS_TX_PIN = 14; // ESP32 TX  -> GPS RX (optional)

// ===================== SD FILE =====================
static const char* ROUTE_PATH = "/route.csv";

// ===================== BLE =====================
static const char* BLE_NAME      = "DogCollar";
static const char* SERVICE_UUID  = "12345678-1234-1234-1234-1234567890ab";
static const char* META_UUID     = "12345678-1234-1234-1234-1234567890ac";
static const char* CTRL_UUID     = "12345678-1234-1234-1234-1234567890ad";
static const char* DATA_UUID     = "12345678-1234-1234-1234-1234567890ae";

// ===================== GLOBALS =====================
TinyGPSPlus gps;
HardwareSerial gpsSerial(1);

File routeFile;

// BLE objects
NimBLECharacteristic* metaChar = nullptr;
NimBLECharacteristic* dataChar = nullptr;

// Stream settings
static const uint16_t CHUNK_BYTES = 180;
static bool streaming = false;
static uint32_t fileSize = 0;

// Logging control
static uint32_t lastLogMs = 0;
static const uint32_t LOG_PERIOD_MS = 1000; // 1 point/sec
static bool sdOk = false;

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

// ---------- SD helpers ----------
bool sdInit() {
  // Use board/core default SPI pins (this is what worked)
  SPI.begin();

  // Optional: make CS a known output level before SD.begin
  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, HIGH);

  // Now init SD
  return SD.begin(SD_CS);
}

void ensureRouteHeader() {
  if (!SD.exists(ROUTE_PATH)) {
    File f = SD.open(ROUTE_PATH, FILE_WRITE);
    if (f) {
      f.println("lat,lon,t_ms");
      f.close();
    }
  }
}

void appendRoutePoint(double lat, double lon, uint32_t t_ms) {
  File f = SD.open(ROUTE_PATH, FILE_APPEND);
  if (!f) return;

  f.print(lat, 6);
  f.print(",");
  f.print(lon, 6);
  f.print(",");
  f.println(t_ms);

  f.close();
}

uint32_t getRouteSize() {
  File f = SD.open(ROUTE_PATH, FILE_READ);
  if (!f) return 0;
  uint32_t sz = (uint32_t)f.size();
  f.close();
  return sz;
}

// ---------- BLE file streaming ----------
bool openRouteForStream() {
  if (!SD.exists(ROUTE_PATH)) return false;
  routeFile = SD.open(ROUTE_PATH, FILE_READ);
  if (!routeFile) return false;

  fileSize = (uint32_t)routeFile.size();
  setMeta(fileSize);
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
      Serial.println("CTRL: START");

      // Pause logging while streaming to avoid SD read/write collision
      streaming = false;
      closeRouteStream();

      if (!sdOk) {
        Serial.println("SD not OK - cannot stream.");
        return;
      }

      if (!openRouteForStream()) {
        Serial.println("Failed to open /route.csv for streaming.");
        return;
      }

      streaming = true;
    } else if (v == "STOP") {
      Serial.println("CTRL: STOP");
      streaming = false;
      closeRouteStream();
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

  // Initialize meta now
  if (sdOk && SD.exists(ROUTE_PATH)) {
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

// ---------- GPS logging loop ----------
void handleGPS() {
  if (streaming) return; // don't write SD while streaming
  if (!sdOk) return;

  while (gpsSerial.available()) {
    gps.encode((char)gpsSerial.read());
  }

  if (gps.location.isValid() && gps.location.isUpdated()) {
    uint32_t now = millis();
    if (now - lastLogMs >= LOG_PERIOD_MS) {
      lastLogMs = now;

      double lat = gps.location.lat();
      double lon = gps.location.lng();

      appendRoutePoint(lat, lon, now);

      // Update BLE meta (file grows)
      setMeta(getRouteSize());

      Serial.print("LOG: ");
      Serial.print(lat, 6);
      Serial.print(",");
      Serial.println(lon, 6);
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(400);
  Serial.println("\n=== Route Sync Firmware (Thing Plus: GPS->SD + BLE stream) ===");

  // SD init
  Serial.println("Init SD...");
  sdOk = sdInit();
  if (!sdOk) {
    Serial.println("❌ SD init failed");
  } else {
    Serial.println("✅ SD init OK");
    ensureRouteHeader();
    Serial.print("Has /route.csv? ");
    Serial.println(SD.exists(ROUTE_PATH) ? "YES" : "NO");
  }

  // GPS init (NEO-6M default 9600)
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.println("GPS serial started @9600 on RX=27 TX=14");

  // BLE init
  setupBLE();
}

void loop() {
  handleGPS();

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

  delay(5);
}
