#include<MPU6050.h>
#include<Wire.h>
MPU6050 mpu;
int dir=33;
int pul=32;
int EnR=19;
void setup() {
    pinMode(dir, OUTPUT);
  pinMode(pul, OUTPUT);
  pinMode(EnR, OUTPUT);
  digitalWrite(pul, LOW);
  digitalWrite(dir, LOW);
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

  float angle = atan2(ax,az)*180/PI; // give angle of rotation around y axis ignoring movement in x axis
  Serial.print("angle: ");Serial.println(angle);
 delayMicroseconds(10);

// considering horizontal position as -2 to 2 degree(perfectly balanced at 0,putting range just for testing), front tilt as +ve angle and back tilt as -ve angle

if(angle>2){              //front tilt counter motor movement
  
 digitalWrite(EnR,HIGH);
digitalWrite(dir,HIGH);
digitalWrite(pul, HIGH);
delayMicroseconds(800);   // will adjust later
digitalWrite(pul, LOW);
delayMicroseconds(800);   // will adjust later

}else if(angle<-2)   //back tilt motor movement
{
   digitalWrite(EnR,HIGH);
  digitalWrite(dir,LOW);
digitalWrite(pul, HIGH);
delayMicroseconds(800);   // will adjust later
digitalWrite(pul, LOW);
delayMicroseconds(800);   // will adjust later

}else{  // perfectl balnced state.. hold position
  digitalWrite(EnR,LOW);
}
}
