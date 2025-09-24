#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

void setup() {
  Serial.begin(115200);

  // I2C pins for ESP32
  Wire.begin(21, 22);  // SDA = 21, SCL = 22
  Serial.println("Starting I2C...");

  // Initialize MPU6050
  Serial.println("Initializing MPU6050...");
  mpu.initialize();

  // Test connection
  Serial.println("Testing connection...");
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed! Check wiring.");
    while (1); // Stop here if not detected
  }
  Serial.println("MPU6050 connected!");

  // Calibrate accelerometer and gyroscope
  Serial.println("Calibrating sensors...");
  mpu.CalibrateAccel(6);
  mpu.CalibrateGyro(6);
  mpu.PrintActiveOffsets();
  delay(2000);
}

void loop() {
  // Read raw accel and gyro values
  int16_t ax, ay, az;
  int16_t gx, gy, gz;

  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  float Ax = ax / 16384.0; // convert to g
  float Ay = ay / 16384.0;
  float Az = az / 16384.0;

float pitch = atan2(Ax, sqrt(Ay*Ay + Az*Az)) * 180/PI;
float roll  = atan2(Ay, sqrt(Ax*Ax + Az*Az)) * 180/PI;

Serial.print("Pitch: "); Serial.print(pitch);
Serial.print(" | Roll: "); Serial.println(roll);


  delay(500); // update twice per second
}
