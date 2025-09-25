float gyroX_offset = 0, gyroY_offset = 0, gyroZ_offset = 0;
#include<MPU6050.h>
#include<Wire.h>
MPU6050 mpu;
// imu varibles
float acc_angle_y; 
float gyro_rate_y; 
float current_angle=0; 
float alpha = 0.96; //Complementary filter coefficient (0.98 = trust gyro 98%, accel 2%)
float dt = 0.005; // Loop time (5ms for 200Hz). Adjust based on actual loop time.
unsigned long prev_time = 0;
void calibrateGyro(); 
  void setup(){
  Serial.begin(115200);
  Wire.begin(21,22);
  mpu.initialize(); //initalising IMU
    if(mpu.testConnection()){   //
    Serial.println("MPU6050 connected...");
    } else{
      Serial.println("MPU6050 not connected...Check Connection");
      while(1);
      }
    calibrateGyro();
  }

  void loop(){
  unsigned long current_time = micros();
  dt = (current_time - prev_time) / 1000000.0; // Convert to seconds
  prev_time = current_time;

    readIMU();
  printAngle();
  current_angle = alpha * (current_angle + gyro_rate_y * dt) + (1 - alpha) * acc_angle_y;
  delay(100);
    }

  void readIMU() {
  int16_t ax,ay,az; // accelerometer values
  int16_t gx,gy,gz; // gyroscope values
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  //Will decice pitch or roll based on final oreintation of mpu
  acc_angle_y = atan2(-ax, sqrt(ay*ay + az*az)) * 180/PI;  //read angle from accelerometer // roll 
  gyro_rate_y = (gy - gyroY_offset) / 131.0;// 131 LSB/(deg/s) for 250deg/s range
  }
  void printAngle(){
     //Will decice pitch or roll based on final oreintation of mpu
   // Serial.print("angle: ");
    //Serial.println(acc_angle_y);
    Serial.println(current_angle);
   // Serial.print("  ");
  }
  void calibrateGyro() {
  Serial.println("Calibrating gyro... KEEP IMU STILL!");
  delay(2000);
  
  long gx_sum = 0, gy_sum = 0, gz_sum = 0;
  int samples =1000;
  for (int i = 0; i < samples; i++) {
    int16_t gx, gy, gz;
    mpu.getRotation(&gx, &gy, &gz);
    gx_sum += gx;
    gy_sum += gy;
    gz_sum += gz;
    delay(2);
  }
  
  gyroX_offset = gx_sum / samples;
  gyroY_offset = gy_sum / samples;
  gyroZ_offset = gz_sum / samples;
  
  Serial.print("Gyro Offsets - X: "); Serial.print(gyroX_offset);
  Serial.print(" Y: "); Serial.print(gyroY_offset);
  Serial.print(" Z: "); Serial.println(gyroZ_offset);
}
