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
  Serial.print("Accel: ");
  Serial.print("ax: ");Serial.print(ax); Serial.print(", ");
  Serial.print("ay: ");Serial.print(ay); Serial.print(", ");
  Serial.print("az: ");Serial.print(az); Serial.print(" | ");
   Serial.print("Gyro: ");
  Serial.print("gx: ");Serial.print(gx); Serial.print(", ");
  Serial.print("gy: ");Serial.print(gy); Serial.print(", ");
  Serial.print("gz: ");Serial.println(gz);
  delay(250); 
}
