#include "Wire.h"


int16_t rax, ray, raz, rgx, rgy, rgz;
double ax, ay, az, axcal, aycal, azcal;
double vector;
double ayaw, apitch, aroll, gyaw, gpitch, groll, gyawcal, gpitchcal, grollcal, ayawcal, apitchcal, arollcal;
double fyaw, fpitch, froll;
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
    Wire.requestFrom(0x68, 6);
    rgx = rgx + Wire.read()<<8|Wire.read();
    rgy = rgy + Wire.read()<<8|Wire.read();
    rgz = rgz + Wire.read()<<8|Wire.read();
    while (ltime > micros());
    ltime = micros() + 4000;
  }
  rgx = rgx/500.0;
  rgy = rgy/500.0;
  rgz = rgz/500.0;
  grollcal = -rgx*0.000031;
  gpitchcal = -rgy*0.000031;
  gyawcal = -rgz*0.000031;
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
  apitchcal = atan(ay/(sqrt(ax*ax + az*az)))*-57.3;
  arollcal = atan(ax/(sqrt(ay*ay + az*az)))*-57.3;
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
  ax = rax/8200.0;
  ay = ray/8200.0;
  az = raz/8200.0;
  apitch = apitchcal + atan(ay/(sqrt(ax*ax + az*az)))*57.3;
  aroll = arollcal + atan(ax/(sqrt(ay*ay + az*az)))*57.3;
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  rgx = Wire.read()<<8|Wire.read();
  rgy = Wire.read()<<8|Wire.read();
  rgz = Wire.read()<<8|Wire.read();
  groll = froll + grollcal + rgx*0.000031;
  gpitch = fpitch + gpitchcal + rgy*0.000031;
  gyaw = gyaw + gyawcal + rgz*0.000031;
  fyaw = gyaw;
  fpitch = gpitch*0.9995 + apitch*0.0005;
  froll = groll*0.9995 + aroll*0.0005;
  // vector = sqrt(ax*ax + ay*ay + az*az);
  // ayaw = acos(ax/vector)*57.2956;
  // apitch = acos(ay/vector)*57.2956;
  // aroll = acos(az/vector)*57.2956;
  // Serial.print(micros()-start);
  // Serial.print(" ");
  // Serial.print(groll);
  // Serial.print(" ");
  // Serial.print(gpitch);
  // Serial.print(" ");
  // Serial.println(gyaw);
  // Serial.print(micros()-start);
  // Serial.print(" ");
  // Serial.print(ayaw);
  // Serial.print(" ");
  // Serial.print(apitch);
  // Serial.print(" ");
  // Serial.println(aroll);
  Serial.print(micros()-start);
  Serial.print(" ");
  Serial.print(fyaw);
  Serial.print(" ");
  Serial.print(fpitch);
  Serial.print(" ");
  Serial.println(froll);
  while (ltime > micros());
  ltime = micros() + 4000;
}
