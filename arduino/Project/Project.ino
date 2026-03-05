#include <TinyGPS++.h>

TinyGPSPlus gps;

// On crée un port série matériel UART2
HardwareSerial gpsSerial(2);

// On définit les pins
#define RX_PIN A3
#define TX_PIN A4

float latitude, longitude;

void setup() {


  //Serial.begin(115200);   // Moniteur série USB
  setupCommunication();

  // Initialisation UART2
  gpsSerial.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);

  Serial.println("GPS prêt...");
}

void loop()
{
  while (gpsSerial.available())
  {
    char c = gpsSerial.read();

    if (gps.encode(c))
    {
      
      
        latitude = gps.location.lat();
        longitude = gps.location.lng();

       /*
        Serial.print("Latitude: ");
        Serial.println(latitude, 6);

        Serial.print("Longitude: ");
        Serial.println(longitude, 6);
        Serial.println("-------------------");
        */
        String sender = String(latitude, 6)+","+String(longitude, 6);
        sendMessage(sender);

      
    }
  }
}