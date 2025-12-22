#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>

#define SDA_PIN 21
#define SCL_PIN 22
#define IMU_ADDR 0x68
#define WHO_AM_I 0x75

MPU6050 mpu(IMU_ADDR);

static constexpr float ACC_COUNTS_PER_G = 16384.0f;   
static constexpr float G_TO_MS2 = 9.80665f;
static constexpr float ACC_SCALE_MS2_PER_COUNT = G_TO_MS2 / ACC_COUNTS_PER_G;

static constexpr float GYRO_COUNTS_PER_DPS = 131.0f;  
static constexpr float GYRO_SCALE_RAD_PER_SEC =
    DEG_TO_RAD / GYRO_COUNTS_PER_DPS;

static uint8_t readWhoAmI() {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(WHO_AM_I);
  if (Wire.endTransmission(false) != 0) return 0xFF;
  if (Wire.requestFrom(IMU_ADDR, 1u) != 1) return 0xFE;
  return Wire.read();
}

void setup() {
  Serial.begin(115200);
  delay(1500);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);

  uint8_t id = readWhoAmI();
  if (id != 0x70) {
    Serial.print("Unexpected WHO_AM_I: 0x");
    Serial.println(id, HEX);
    while (1) delay(100);
  }

  mpu.initialize();
  delay(50);

  Serial.println("Printing accel (m/s^2) and gyro (rad/s)");
}

void loop() {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Accelerometer → m/s^2
  float ax_ms2 = ax * ACC_SCALE_MS2_PER_COUNT;
  float ay_ms2 = ay * ACC_SCALE_MS2_PER_COUNT;
  float az_ms2 = az * ACC_SCALE_MS2_PER_COUNT;

  // Gyroscope → rad/s
  float gx_rads = gx * GYRO_SCALE_RAD_PER_SEC;
  float gy_rads = gy * GYRO_SCALE_RAD_PER_SEC;
  float gz_rads = gz * GYRO_SCALE_RAD_PER_SEC;

  Serial.print("A [m/s^2]: ");
  Serial.print(ax_ms2, 4); Serial.print(", ");
  Serial.print(ay_ms2, 4); Serial.print(", ");
  Serial.print(az_ms2, 4);

  Serial.print(" | G [rad/s]: ");
  Serial.print(gx_rads, 4); Serial.print(", ");
  Serial.print(gy_rads, 4); Serial.print(", ");
  Serial.println(gz_rads, 4);

  delay(200);
}
