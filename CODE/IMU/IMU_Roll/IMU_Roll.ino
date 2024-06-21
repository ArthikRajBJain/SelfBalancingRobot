#include "Wire.h"


int16_t rax, ray, raz, rgx;
double ax, ay, az, axcal, aycal, azcal;
double aroll, groll, grollcal, arollcal;
double froll;
double ltime;

void setup() 
{
  Wire.begin();
  Wire.setClock(400000);
  Serial.begin(115200);
  //POWER MANAGEMENT
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  //GYROSCOPE RANGE(+/-250dps)
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x00);
  Wire.endTransmission();
  //ACCELEROMETER RANGE(+/-4g)
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x08);
  Wire.endTransmission();
  //FILTERING
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x03);
  
  Wire.endTransmission();
  ltime = micros();
  for(int i=0;i<500;i++)
  {
    Wire.beginTransmission(0x68);
    Wire.write(0x43);
    Wire.endTransmission();
    Wire.requestFrom(0x68, 2);
    rgx = rgx + Wire.read()<<8|Wire.read();
    while (ltime > micros());
    ltime = micros() + 4000;
  }
  rgx = rgx/500.0;
  grollcal = -rgx*0.000031;
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  rax = Wire.read()<<8|Wire.read();
  ray = Wire.read()<<8|Wire.read();
  raz = Wire.read()<<8|Wire.read();
  ax = rax/8200.0;
  ay = ray/8200.0;
  az = raz/8200.0;
  // arollcal = atan(ax/(sqrt(ay*ay + az*az)))*-57.3;
  arollcal = atan(ay/(sqrt(ax*ax + az*az)))*62.1325;
}

double start;

void loop() 
{
  start = micros();
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  rax = Wire.read()<<8|Wire.read();
  ray = Wire.read()<<8|Wire.read();
  raz = Wire.read()<<8|Wire.read();
  ax = rax/8192.0;
  ay = ray/8192.0;
  az = raz/8192.0;
  if(az >= 0) aroll = -(arollcal - atan(ay/(sqrt(ax*ax + az*az)))*62.1325);
  else aroll = (arollcal - atan(ay/(sqrt(ax*ax + az*az)))*62.1325);
  // aroll = atan(ay/(sqrt(ax*ax + az*az)))*62.1325;
  // aroll = asin((double)ray/8200.0)*57.296;
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 2);
  rgx = Wire.read()<<8|Wire.read();
  groll = froll + grollcal + rgx*0.000031;
  froll = groll*0.995 + aroll*0.005;
  Serial.print(micros()-start);
  Serial.print(" ");
  Serial.print(aroll);
  Serial.print(" ");
  Serial.print(az);
  Serial.print(" ");
  Serial.print(groll);
  Serial.print(" ");
  Serial.println(froll);
  // Serial.print(-90);
  // Serial.print(" ");
  // Serial.print(froll);
  // Serial.print(" ");
  // Serial.println(90);
  while (ltime > micros());
  ltime = micros() + 4000;
}
