#include <Arduino.h>
#include <Wire.h>

#define SDA_PIN 21
#define SCL_PIN 22
#define ADDR    0x68

static void write8(uint8_t reg, uint8_t val){
  Wire.beginTransmission(ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

static int16_t read16(uint8_t reg){
  Wire.beginTransmission(ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(ADDR, (uint8_t)2);
  uint8_t hi = Wire.read();
  uint8_t lo = Wire.read();
  return (int16_t)((hi << 8) | lo);
}

void setup() {
  Serial.begin(115200);
  delay(1500);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000); 

  write8(0x6B, 0x01);

  write8(0x1C, 0x00); 

  // Gyro config (0x1B): 0 = ±250 dps, 1=±500, 2=±1000, 3=±2000
  write8(0x1B, 0x00); // ±250 dps

  Serial.println("IMU OK (raw MPU-style)");
}

void loop() {
  int16_t ax = read16(0x3B);
  int16_t ay = read16(0x3D);
  int16_t az = read16(0x3F);

  int16_t gx = read16(0x43);
  int16_t gy = read16(0x45);
  int16_t gz = read16(0x47);

  // Sensitivity (MPU-style): ±2g => 16384 LSB/g, ±250 dps => 131 LSB/(deg/s)
  float ax_g = ax / 16384.0f;
  float ay_g = ay / 16384.0f;
  float az_g = az / 16384.0f;

  float gx_dps = gx / 131.0f;
  float gy_dps = gy / 131.0f;
  float gz_dps = gz / 131.0f;

  Serial.print("A[g]: ");
  Serial.print(ax_g, 3); Serial.print(", ");
  Serial.print(ay_g, 3); Serial.print(", ");
  Serial.print(az_g, 3);

  Serial.print(" | G[dps]: ");
  Serial.print(gx_dps, 2); Serial.print(", ");
  Serial.print(gy_dps, 2); Serial.print(", ");
  Serial.println(gz_dps, 2);

  delay(100);
}
