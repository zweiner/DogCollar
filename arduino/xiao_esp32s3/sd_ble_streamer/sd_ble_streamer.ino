#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <NimBLEDevice.h>

// ---- Pins (your proven-good wiring) ----
static const int PIN_SCK  = D8;
static const int PIN_MISO = D9;
static const int PIN_MOSI = D10;
static const int PIN_CS   = D4;

// ---- SD file ----
static const char* ROUTE_PATH = "/route.csv";

// ---- BLE ----
static const char* BLE_NAME    = "DogCollar_V1";
static const char* SERVICE_UUID = "12345678-1234-1234-1234-1234567890ab";
static const char* META_UUID    = "12345678-1234-1234-1234-1234567890ac";
static const char* CTRL_UUID    = "12345678-1234-1234-1234-1234567890ad";
static const char* DATA_UUID    = "12345678-1234-1234-1234-1234567890ae";

SPIClass spi(FSPI);
File routeFile;

NimBLECharacteristic* metaChar = nullptr;
NimBLECharacteristic* dataChar = nullptr;

// Chunk size per notification (keep conservative for reliability)
static const uint16_t CHUNK_BYTES = 180;

static bool streaming = false;
static uint32_t fileSize = 0;
static uint32_t sentBytes = 0;

#pragma pack(push, 1)
struct MetaPacket {
  uint32_t file_size;
  uint16_t chunk_bytes;
};
#pragma pack(pop)

void setMeta(uint32_t sizeBytes) {
  MetaPacket m{ sizeBytes, CHUNK_BYTES };
  metaChar->setValue((uint8_t*)&m, sizeof(m));
}

bool sdInit() {
  spi.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);
  return SD.begin(PIN_CS, spi, 1000000); // 1MHz (known good)
}

bool openRoute() {
  if (!SD.exists(ROUTE_PATH)) return false;
  routeFile = SD.open(ROUTE_PATH, FILE_READ);
  if (!routeFile) return false;
  fileSize = (uint32_t)routeFile.size();
  sentBytes = 0;
  setMeta(fileSize);
  return true;
}

void closeRoute() {
  if (routeFile) routeFile.close();
}

class CtrlCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* c, NimBLEConnInfo& connInfo) override {
    (void)connInfo;
    std::string v = c->getValue();

    if (v == "START") {
      Serial.println("CTRL: START");
      closeRoute();
      if (!openRoute()) {
        Serial.println("Failed to open /route.csv");
        streaming = false;
        return;
      }
      streaming = true;
    } else if (v == "STOP") {
      Serial.println("CTRL: STOP");
      streaming = false;
      closeRoute();
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

  // Initialize META even before streaming
  if (SD.exists(ROUTE_PATH)) {
    File f = SD.open(ROUTE_PATH, FILE_READ);
    if (f) { setMeta((uint32_t)f.size()); f.close(); }
  } else {
    setMeta(0);
  }

  svc->start();

NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
adv->addServiceUUID(SERVICE_UUID);
adv->start();

  Serial.println("BLE advertising started (DogCollar).");
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== SD -> BLE Route Streamer ===");

  Serial.println("Init SD...");
  if (!sdInit()) {
    Serial.println("❌ SD init failed");
  } else {
    Serial.println("✅ SD init OK");
    Serial.print("Has /route.csv? ");
    Serial.println(SD.exists(ROUTE_PATH) ? "YES" : "NO");
  }

  setupBLE();
}

void loop() {
  if (!streaming) {
    delay(10);
    return;
  }
  if (!routeFile) {
    streaming = false;
    return;
  }

  uint8_t buf[CHUNK_BYTES];
  int n = routeFile.read(buf, CHUNK_BYTES);

  if (n > 0) {
    dataChar->setValue(buf, n);
    dataChar->notify();
    sentBytes += (uint32_t)n;

    // Progress to serial (optional)
    // Serial.printf("sent %lu/%lu\n", (unsigned long)sentBytes, (unsigned long)fileSize);

    delay(8); // pacing helps reliability
  } else {
    Serial.println("Stream complete.");
    streaming = false;
    closeRoute();
  }
}
