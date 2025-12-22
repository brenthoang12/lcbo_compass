#include <Arduino.h>
#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

static bool btScanAsync = true;

void btAdvertisedDeviceFound(BTAdvertisedDevice *pDevice) {
  Serial.printf("Found device: %s\n", pDevice->toString().c_str());
}

void setup() {
  Serial.begin(115200);

  SerialBT.begin("ESP32test");
  Serial.println("Bluetooth started. Pair/connect to ESP32test.");

  if (btScanAsync) {
    Serial.println("Starting async discovery for 5s...");
    if (SerialBT.discoverAsync(btAdvertisedDeviceFound)) {
      delay(5000);
      SerialBT.discoverAsyncStop();
      Serial.println("Discovery stopped.");
    } else {
      Serial.println("discoverAsync failed (often fails after connect).");
    }
  }
}

void loop() {

  static uint32_t last = 0;
  if (millis() - last >= 1000) {
    last = millis();
    int adc = analogRead(34);
    SerialBT.printf("t_ms=%lu, adc=%d\n", (unsigned long)millis(), adc);
    // print to USB serial for debugging
    Serial.printf("Sent BT: t_ms=%lu, adc=%d\n", (unsigned long)millis(), adc);
  }


  if (SerialBT.available()) {
    String cmd = SerialBT.readStringUntil('\n');
    cmd.trim();
    Serial.print("Got BT cmd: ");
    Serial.println(cmd);
    SerialBT.print("ACK: ");
    SerialBT.println(cmd);
  }
}
