#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GPS.h>

// GPS over I2C
Adafruit_GPS GPS(&Wire);

char nmeaLine[120];
uint8_t idx = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin();              // ESP32: Wire.begin(SDA, SCL);

  Serial.println("Adafruit GPS I2C line reader");
  if (!GPS.begin(0x10)) {
    Serial.println("GPS.begin failed (check I2C address/wiring)");
    while (1) delay(10);
  }
}

void loop() {
  while (GPS.available()) {
    char c = GPS.read();
    if (c == 0) continue;
    if (c == '\r') continue;
    if (c == '\n') {
      nmeaLine[idx] = '\0';
      idx = 0;
      if (nmeaLine[0] == '$') {
        Serial.println(nmeaLine);
        GPS.parse(nmeaLine);

        // Optional: show fix status
        Serial.print("fix=");
        Serial.print(GPS.fix ? "YES" : "NO");
        Serial.print(" sats=");
        Serial.println((int)GPS.satellites);
      }
    } else {
      // Prevent buffer overflow
      if (idx < sizeof(nmeaLine) - 1) {
        nmeaLine[idx++] = c;
      } else {
        // line too long, reset
        idx = 0;
      }
    }
  }
}
