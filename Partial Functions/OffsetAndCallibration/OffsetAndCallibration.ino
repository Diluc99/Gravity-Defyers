float gyroX_offset = 0, gyroY_offset = 0, gyroZ_offset = 0;
#include<MPU6050.h>
#include<Wire.h>
MPU6050 mpu;
// imu varibles
float acc_angle_y; //roll
float acc_angle_x; //pitch
// have to put in setup
void calibrateGyro(int samples = 1000); 
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
    readIMU();
  printAngle();
    }

  void readIMU() {
  int16_t ax,ay,az; // accelerometer values
  int16_t gx,gy,gz; // gyroscope values
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  //Will decice pitch or roll based on final oreintation of mpu
  acc_angle_y= atan2(ax,az)*180/PI;  //read angle from accelerometer // roll 
  acc_angle_x= atan2(ay,az)*180/PI;  //read angle from accelerometer // pitch
  }
  void printAngle(){
     //Will decice pitch or roll based on final oreintation of mpu
    Serial.print("roll: ");
    Serial.println(acc_angle_y);
    Serial.print("pitch: ");
    Serial.println(acc_angle_x);
  }
  void calibrateGyro(int samples) {
  Serial.println("Calibrating gyro... KEEP IMU STILL!");
  delay(2000);
  
  long gx_sum = 0, gy_sum = 0, gz_sum = 0;
  
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
