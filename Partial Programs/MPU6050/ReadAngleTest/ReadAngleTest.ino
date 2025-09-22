#include<MPU6050.h>
#include<Wire.h>
MPU6050 mpu;
void setup() {
Serial.begin(115200);
Wire.begin(21,22);
mpu.initialize();
if(mpu.testConnection()){
  Serial.println("Connected");
}else{
  Serial.println("Not Connected");
  while(1);
}

}

void loop() {
  // put your main code here, to run repeatedly:
  int16_t ax,ay,az; // accelerometer values
  int16_t gx,gy,gz; // gyroscope values
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); // reading values from MPU6050

  float angle = atan2(ax,az); // give angle of rotation around y axis ignoring movement in x axis
  Serial.print("angle: ");Serial.println(angle);
  delay(250); 
}
