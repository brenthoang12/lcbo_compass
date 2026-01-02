// TODO: recalibrate accelerometer
// TODO: reduce variables for accel and gyro
// TODO: add comment for gyro calibration 
// TODO: add warning for different IMU orientation

#include <Wire.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_LSM6DS3TRC.h>
#include <Adafruit_Sensor.h>

Adafruit_LIS3MDL lis3mdl;
Adafruit_LSM6DS3TRC lsm6ds3trc;

// accelerometer calibration
float xMax = 1.01f;
float xMin = -1.00f;
float yMax = 1.00f;
float yMin = -0.99f;
float zMax = 1.01f;
float zMin = -0.99f;

float xOffset = (xMax + xMin) / 2.0f;
float yOffset = (yMax + yMin) / 2.0f;
float zOffset = (zMax + zMin) / 2.0f;
float xScale = 2.0f / (xMax - xMin);
float yScale = 2.0f / (yMax - yMin);
float zScale = 2.0f / (zMax - zMin);

float roll_raw;
float pitch_raw;
float roll = 0.0f;
float pitch = 0.0f;

// gyro
float roll_raw_g; 
float pitch_raw_g;
float yaw_raw_g;
float roll_g = 0.0f;
float pitch_g = 0.0f;;
float yaw_g = 0.0f;
float xOffset_g = 0.0f;
float yOffset_g = 0.0f;
float zOffset_g = 0.0f;

unsigned long tStart = 0;

const float DECLINATION_DEG = -10.17f;
const float DECLINATION_RAD = DECLINATION_DEG * (PI / 180.0f);
// const float PHONE_OFFSET_DEG = 0;
const float G = 9.81f;  // 9.80665f

const float HARD_IRON[3] = { -35.92f, 13.33f, -20.58f };
const float SOFT_IRON[3][3] = {
  { 0.973f, 0.047f, -0.109f },
  { 0.047f, 1.056f, 0.006f },
  { -0.109f, 0.006f, 0.987f }
};

void calibrate_gyro() {
  Serial.println("Calibrating gyro... Please keep the sensor still.");
  delay(1000);
  int calibration_samples = 1000;
  float sumX = 0.0f;
  float sumY = 0.0f;
  float sumZ = 0.0f;

  for (int i = 0; i < calibration_samples; i++) {
    sensors_event_t accel, gyro, temp;
    lsm6ds3trc.getEvent(&accel, &gyro, &temp);
    sumX += gyro.gyro.y;
    sumY += gyro.gyro.x;
    sumZ += -gyro.gyro.z;
    delay(10);
  }
  xOffset_g = sumX / calibration_samples; 
  yOffset_g = sumY / calibration_samples;
  zOffset_g = sumZ / calibration_samples;
  
}

void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  if (!lis3mdl.begin_I2C()) {
    Serial.println("Failed to find LIS3MDL chip");
    while (1) {
      delay(10);
    }
  }

  if (!lsm6ds3trc.begin_I2C()) {
    Serial.println("Failed to find LSM6DS3TR-C chip");
    while (1) {
      delay(10);
    }
  }

  lis3mdl.setPerformanceMode(LIS3MDL_MEDIUMMODE);
  lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
  lis3mdl.setDataRate(LIS3MDL_DATARATE_155_HZ);
  lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);
  lis3mdl.setIntThreshold(500);
  lis3mdl.configInterrupt(false, false, true, true, false, true);

  lsm6ds3trc.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
  lsm6ds3trc.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);  // default 2000 DPS

  lsm6ds3trc.setAccelDataRate(LSM6DS_RATE_104_HZ);  // default 104 Hz
  lsm6ds3trc.setGyroDataRate(LSM6DS_RATE_104_HZ);   // default 104 Hz

  lsm6ds3trc.configInt1(false, false, true);
  lsm6ds3trc.configInt2(false, true, false);

  tStart = millis();
  calibrate_gyro();
}

void loop() {
  sensors_event_t mag, accel, gyro, temp;
  lis3mdl.getEvent(&mag);
  lsm6ds3trc.getEvent(&accel, &gyro, &temp);

  // accelerometer

  float a_raw[3] = { accel.acceleration.y, accel.acceleration.x, accel.acceleration.z };

  float a_G[3] = {
    ((a_raw[0] / G)),
    ((a_raw[1] / G)),
    ((a_raw[2] / G))
  };

  float a_G_cal[3] = {
    (a_G[0] - xOffset) * xScale,
    (a_G[1] - yOffset) * yScale,
    (a_G[2] - zOffset) * zScale
  };

  float a_cal[3] = {
    a_G_cal[0] * G,
    a_G_cal[1] * G,
    a_G_cal[2] * G
  };

  roll_raw  = atan2(-a_cal[1], sqrt(a_cal[2] * a_cal[2] + a_cal[0] * a_cal[0])) * 360 / (2 * 3.14);
  pitch_raw = atan2(a_cal[0], sqrt(a_cal[2] * a_cal[2] + a_cal[1] * a_cal[1])) * 360 / (2 * 3.14);
  roll = .8 * roll + .2 * roll_raw;
  pitch = .8 * pitch + .2 * pitch_raw;

  // gyro
  float g_raw[3] = { gyro.gyro.y, gyro.gyro.x, -gyro.gyro.z }; 
  float g_cal[3] = {
    g_raw[0] - xOffset_g,
    g_raw[1] - yOffset_g,
    g_raw[2] - zOffset_g
  };

  float dt = (millis() - tStart) * 0.001f;
  tStart = millis();

  float rotation_rad_g[3] = {
    roll_g  + dt * g_cal[0],
    pitch_g + dt * g_cal[1],
    yaw_g   + dt * g_cal[2]
  };

  for (int i = 0; i < 3; i++) {
    if (rotation_rad_g[i] > PI)  rotation_rad_g[i] -= 2.0f * PI;
    if (rotation_rad_g[i] < -PI) rotation_rad_g[i] += 2.0f * PI;
  }

  float rotation_deg_g[3] = {
    rotation_rad_g[0] * 180.0f / (float)PI,
    rotation_rad_g[1] * 180.0f / (float)PI,
    rotation_rad_g[2] * 180.0f / (float)PI
  };

  roll_g  = rotation_rad_g[0];
  pitch_g = rotation_rad_g[1];
  yaw_g   = rotation_rad_g[2];

  float m_raw[3] = { mag.magnetic.x, mag.magnetic.y, mag.magnetic.z };
  
  Serial.print("Gx:");
  Serial.print(g_cal[0], 2);
  Serial.print("\tGy:");
  Serial.print(g_cal[1], 2);
  Serial.print("\tGz:");
  Serial.print(g_cal[2], 2);
  Serial.print("\tUL:");
  Serial.print(6);
  Serial.print("\tLL:");
  Serial.println(-6);


  // Serial.print("Roll:");
  // Serial.print(rotation_deg_g[0], 1);
  // Serial.print("\tPitch:");
  // Serial.print(rotation_deg_g[1], 1);
  // Serial.print("\tYaw:");
  // Serial.print(rotation_deg_g[2], 1);
  // Serial.print("\tUL:");
  // Serial.print(90);
  // Serial.print("\tLL:");
  // Serial.println(-90);

  // Serial.print("Ax:");
  // Serial.print(a_G_cal[0], 2);
  // Serial.print("\tAy:");
  // Serial.print(a_G_cal[1], 2);
  // Serial.print("\tAz:");
  // Serial.print(a_G_cal[2], 2);
  // Serial.print("\tUL:");
  // Serial.print(1);
  // Serial.print("\tLL:");
  // Serial.println(-1);

  // Serial.print("Roll:");
  // Serial.print(roll, 1);
  // Serial.print("\tPitch:");
  // Serial.print(pitch, 1);
  // Serial.print("\tUL:");
  // Serial.print(90);
  // Serial.print("\tLL:");
  // Serial.println(-90);



  delay(1);
}

// int gyro_data_g[3] = {
//   (int)lroundf(raw_g[0] * (180.0f / PI)),
//   (int)lroundf(raw_g[1] * (180.0f / PI)),
//   (int)lroundf(raw_g[2] * (180.0f / PI))
// };

// int mag_data_i[3] = {
//   (int)lroundf(mag_data[0] * 10.0f),
//   (int)lroundf(mag_data[1] * 10.0f),
//   (int)lroundf(mag_data[2] * 10.0f)
// };
