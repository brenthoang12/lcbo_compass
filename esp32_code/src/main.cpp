#include <Wire.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_LSM6DS3TRC.h>
#include <Adafruit_Sensor.h>

Adafruit_LIS3MDL lis3mdl;
Adafruit_LSM6DS3TRC lsm6ds3trc;


float xMax = 1.00f;
float xMin = -1.00f;
float yMax = 1.00f;
float yMin = -1.01f;
float zMax = 1.01f;
float zMin = -0.99f;

float xOffset = (xMax + xMin) / 2.0f;
float yOffset = (yMax + yMin) / 2.0f;
float zOffset = (zMax + zMin) / 2.0f;
float xScale = 2.0f/(xMax - xMin); 
float yScale = 2.0f/(yMax - yMin); 
float zScale = 2.0f/(zMax - zMin);

float roll_raw;
float pitch_raw;

float roll  = 0.0f;
float pitch = 0.0f;

const float DECLINATION_DEG = -10.17f;
const float DECLINATION_RAD = DECLINATION_DEG * (PI / 180.0f);
// const float PHONE_OFFSET_DEG = 0;  
const float G = 9.81f; // 9.80665f

const float HARD_IRON[3] = {-35.92f, 13.33f, -20.58f};
const float SOFT_IRON[3][3] = {
  {  0.973f,  0.047f, -0.109f },
  {  0.047f,  1.056f,  0.006f },
  { -0.109f,  0.006f,  0.987f }
};

void setup(void)
{
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
  lsm6ds3trc.setGyroRange(LSM6DS_GYRO_RANGE_500_DPS); // default 2000 DPS

  lsm6ds3trc.setAccelDataRate(LSM6DS_RATE_104_HZ); // default 104 Hz
  lsm6ds3trc.setGyroDataRate(LSM6DS_RATE_104_HZ); // default 104 Hz

  lsm6ds3trc.configInt1(false, false, true); 
  lsm6ds3trc.configInt2(false, true, false); 
}

void loop() {
  sensors_event_t mag, accel, gyro, temp;
  lis3mdl.getEvent(&mag);
  lsm6ds3trc.getEvent(&accel, &gyro, &temp);

  float a_raw[3] = {accel.acceleration.x, accel.acceleration.y, accel.acceleration.z};
  float g_raw[3] = {gyro.gyro.x, gyro.gyro.y, gyro.gyro.z};
  float m_raw[3] = {mag.magnetic.x, mag.magnetic.y, mag.magnetic.z};

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
  
  roll_raw = atan2(a_cal[0],sqrt(a_cal[2]*a_cal[2]+a_cal[1]*a_cal[1]))*360/(2*3.14);
  pitch_raw = atan2(a_cal[1],sqrt(a_cal[2]*a_cal[2]+a_cal[0]*a_cal[0]))*360/(2*3.14);
  roll = .8*roll + .2*roll_raw;
  pitch = .8*pitch + .2*pitch_raw;

  Serial.print("Ax: "); Serial.print(a_cal[0], 1);
  Serial.print(" Ay: "); Serial.print(a_cal[1], 1);
  Serial.print(" Az: "); Serial.println(a_cal[2], 1);
  
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

  delay(10);
}
