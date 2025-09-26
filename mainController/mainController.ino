#include<MPU6050.h>
#include<Wire.h>

MPU6050 mpu;  //MPU6050 OBject
// imu varibles
float acc_angle_pitch; 
float gyro_rate_y;
float gyroX_offset = 0, gyroY_offset = 0, gyroZ_offset = 0; 
float current_angle=0; 
float alpha = 0.96; //Complementary filter coefficient (0.96 = trust gyro 96%, accel 4%)
float dt = 0.005; // Loop time (5ms for 200Hz). Adjust based on actual loop time.
unsigned long prev_time = 0;

float target_angle = 0.0; // the position we want to achieve and hold
//PID VARIABLES
float error = 0; 
float previous_error = 0;
float integral = 0; 
float derivative = 0;
float pidVal = 0; 

//PID CONSTANTS(I'VE TO TEST AND SET, keeping 0 for now)
float Kp =15;
float Ki =0;
float Kd =0.5;

// Motor Pins
  #define RIGHT_PUL_PIN 32
  #define RIGHT_DIR_PIN 33
  #define LEFT_PUL_PIN 25
  #define LEFT_DIR_PIN 26
  #define LEFT_EN_PIN 14
  #define RIGHT_EN_PIN 19

  void setup(){
  Serial.begin(115200);
  Wire.begin(21,22);
  mpu.initialize(); //initalising IMU
    if(mpu.testConnection()){   //testing mpu Connection
    Serial.println("MPU6050 connected...");
    } else{
      Serial.println("MPU6050 not connected...Check Connection");
      while(1);
      }
      calibrateGyro();
    pinMode(RIGHT_PUL_PIN,OUTPUT);
    pinMode(RIGHT_DIR_PIN,OUTPUT);
    pinMode(LEFT_PUL_PIN ,OUTPUT);
    pinMode(LEFT_DIR_PIN ,OUTPUT);
    pinMode(LEFT_EN_PIN,OUTPUT);
    pinMode(RIGHT_EN_PIN,OUTPUT);
    digitalWrite(RIGHT_EN_PIN, LOW);   // Enable right motor
    digitalWrite(LEFT_EN_PIN, LOW);
    prev_time = micros();
  }

  void loop(){
    unsigned long current_time = micros();
    dt = (current_time - prev_time) / 1000000.0; // Convert to seconds
    prev_time = current_time;
    readIMU();
    current_angle = alpha * (current_angle + gyro_rate_y * dt) + (1 - alpha) * acc_angle_pitch; //complimentary filter formula
    calculatePID();
    MotorControls();
    //delay(5);
  //  printAllValues(); //debug
  }

  void readIMU() {
    int16_t ax,ay,az; // accelerometer values
    int16_t gx,gy,gz; // gyroscope values
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  acc_angle_pitch = atan2(-ax, sqrt(ay*ay + az*az)) * 180/PI; //read angle from accelerometer (pitch)
  gyro_rate_y = (gy - gyroY_offset) / 131.0;// 131 LSB/(deg/s) for 250deg/s range
  }
  
  void calibrateGyro() {
  Serial.println("Calibrating gyro... KEEP IMU STILL!");
  delay(2000);
  
  long gx_sum = 0, gy_sum = 0, gz_sum = 0;
  int samples =1000; // taking 1000 reading avg for callibration
  for (int i = 0; i < samples; i++) {
    int16_t gx, gy, gz;
    mpu.getRotation(&gx, &gy, &gz);
    gx_sum += gx;  //total sum
    gy_sum += gy;
    gz_sum += gz;
    delay(2);
  }
  // avg = total sum/no of samples
  gyroX_offset = gx_sum / samples;
  gyroY_offset = gy_sum / samples;  
  gyroZ_offset = gz_sum / samples;
  
  Serial.print("Gyro Offsets - X: "); Serial.print(gyroX_offset);
  Serial.print(" Y: "); Serial.print(gyroY_offset);
  Serial.print(" Z: "); Serial.println(gyroZ_offset);
 }
  
 void MotorControls(){
  if(pidVal>0){
   digitalWrite(RIGHT_DIR_PIN,HIGH);
   digitalWrite(LEFT_DIR_PIN,LOW);
}else{
   digitalWrite(RIGHT_DIR_PIN,LOW);
   digitalWrite(LEFT_DIR_PIN,HIGH);
   pidVal=abs(pidVal);
}



if(pidVal > 1.0) {
    int stepsPerSecond = map(pidVal, 1, 100, 50, 1000);  // Fixed mapping
    int stepDelay = 1000000 / stepsPerSecond;  // Convert to microseconds
    
    int pulsesToSend = (stepsPerSecond * dt);
 for(int i = 0; i < pulsesToSend; i++) {
  digitalWrite(RIGHT_PUL_PIN,HIGH);
  digitalWrite(LEFT_PUL_PIN,HIGH);
  delayMicroseconds(5);
  digitalWrite(RIGHT_PUL_PIN,LOW);
  digitalWrite(LEFT_PUL_PIN,LOW);
  delayMicroseconds(stepDelay);
  }
} 
 }

  void calculatePID(){
  error = current_angle - target_angle;
  integral += error * dt;
   // Integral Windup Protection...
  if(integral > 100) integral = 100;
  if(integral < -100) integral = -100;
  derivative = (error - previous_error) / dt;
  previous_error = error;
  pidVal = (Kp * error) + (Ki * integral) + (Kd * derivative);
}
 
void printAllValues(){ // for debugging
    Serial.print("error: ");
    Serial.print(error);
  /*Serial.print("    ");
    Serial.print("integral: ");
    Serial.print(integral);
  Serial.print("    ");
    Serial.print("Derivative: ");
    Serial.print(derivative);
  Serial.print("    ");*/
    Serial.print("angle: ");
    Serial.print(acc_angle_pitch);
  Serial.print("    ");
  Serial.print("Pid value: ");
  Serial.print(pidVal);
  Serial.println("    ");
    Serial.print("Speed: ");
  Serial.println(map(pidVal, 0, 100, 2000, 200));
}