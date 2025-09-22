int dir=33;
int pul=32;
int angle; 
void setup() {
  // put your setup code here, to run once:
  pinMode(dir, OUTPUT);
  pinMode(pul, OUTPUT);
  digitalWrite(pul, LOW);
  digitalWrite(dir, LOW);
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(angle>0)
{
digitalWrite(dir,HIGH);
digitalWrite(pul,HIGH);
}
  else if(anle<0)
{
  digitalWrite(dir,LOW);
digitalWrite(pul,HIGH);
}
