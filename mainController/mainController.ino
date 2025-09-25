#include<MPU6050.h>
#include<Wire.h>

MPU6050 mpu;  //MPU6050 OBject
// imu varibles
float acc_angle_y; 
float gyro_angle_y;
float current_angle; 
float target_angle; 
//float acc_angle_x;

// Motor Pins
  #define RIGHT_PUL_PIN 32
  #define RIGHT_DIR_PIN 33
  #define LEFT_PUL_PIN 25
  #define LEFT_DIR_PIN 26

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
    pinMode(RIGHT_PUL_PIN,OUTPUT);
    pinMode(RIGHT_DIR_PIN,OUTPUT);
    pinMode(LEFT_PUL_PIN ,OUTPUT);
    pinMode(LEFT_DIR_PIN ,OUTPUT);.
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
 // acc_angle_x= atan2(ay,az)*180/PI;  //read angle from accelerometer // pitch
  }
  void printAngle(){
     //Will decice pitch or roll based on final oreintation of mpu
    Serial.print("angle: ");
    Serial.println(acc_angle_y);
   // Serial.print("angle: ");
    //Serial.println(acc_angle_x);
  }
void MotorControls(){
  if(){//pid condtion will add later
   RIGHT_DIR_PIN=HIGH;
   LEFT_DIR_PIN=LOW
}else{
   RIGHT_DIR_PIN=LOW;
   LEFT_DIR_PIN=HIGH
}
  digitalWrite(RIGHT_PUL_PIN,HIGH);
  digitalWrite(LEFT_PUL_PIN,HIGH);
  delay(10);
  digitalWrite(RIGHT_PUL_PIN,LOW;
  digitalWrite(LEFT_PUL_PIN,LOW);
  delay(####); //speed will map with pid 
}
