#include <WiFi.h>
#include <ESPmDNS.h>
#include <NetworkUdp.h>
#include <ArduinoOTA.h>
#include "BluetoothSerial.h"
#include "Wire.h"
#include "PID.h"
#include "math.h"

const char *ssid = "XXXXXXX";
const char *password = "XXXXXXX";

String device_name = "SelfBalancingRobot";
hw_timer_t *timer = NULL;
double ltime = 0;
double LSpeed = 0, RSpeed = 0, LMotor = 0, RMotor = 0, LMotorCount = 0, RMotorCount = 0;
bool LDirection = 0, RDirection = 0;
bool LITRFlag = 0, RITRFlag = 0;
double input, output, setpoint = 0, Kp = 50, Ki = 0.005, Kd = 0.0001, setpointMem = 0;
bool direction = 0;
int16_t rax, ray, raz, rgx;
double ax, ay, az, axcal, aycal, azcal;
double aroll, groll, grollcal, arollcal;
double froll;

PID pid(&input, &output, &setpoint, Kp, Ki, Kd, direction, micros());

void IRAM_ATTR onTimer() {
  LMotorCount++;
  if (LITRFlag == 1) {
    digitalWrite(25, LOW);
    LITRFlag = 0;
    LMotorCount = 0;
  }
  if (LMotorCount > LMotor) {
    digitalWrite(25, HIGH);
    LITRFlag = 1;
  }

  RMotorCount++;
  if (RITRFlag == 1) {
    digitalWrite(14, LOW);
    RITRFlag = 0;
    RMotorCount = 0;
  }
  if (RMotorCount > RMotor) {
    digitalWrite(14, HIGH);
    RITRFlag = 1;
  }
}

BluetoothSerial SerialBT;

int numFinder(double speed) {
  return int(round((1218.75 / speed) - 2.0));
}

TaskHandle_t task_loop1;
void esploop1(void *pvParameters) {
  setup1();

  for (;;) {
    loop1();
  }
}

void setup() {
  // Motor N
  pinMode(33, OUTPUT);  //INA
  pinMode(25, OUTPUT);  //STP
  pinMode(26, OUTPUT);  //DIR
  // Motor P
  pinMode(12, OUTPUT);  //INA
  pinMode(14, OUTPUT);  //STP
  pinMode(27, OUTPUT);  //DIR
  digitalWrite(33, HIGH);
  digitalWrite(12, HIGH);
  digitalWrite(26, LOW);
  digitalWrite(27, HIGH);
  timer = timerBegin(1000000);
  timerAttachInterrupt(timer, &onTimer);
  timerAlarm(timer, 20, true, 0);
  xTaskCreatePinnedToCore(
    esploop1,               /* Task function. */
    "loop1",                /* name of task. */
    10000,                  /* Stack size of task */
    NULL,                   /* parameter of the task */
    1,                      /* priority of the task */
    &task_loop1,            /* Task handle to keep track of created task */
    !ARDUINO_RUNNING_CORE); /* pin task to core @ */
  Serial.begin(115200);
  ltime = micros() + 4000;
  pid.setOutputLimits(-613, 613);
  pid.updateTime(micros());
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
  pinMode(33, OUTPUT);  //INA
  pinMode(25, OUTPUT);  //STP
  pinMode(26, OUTPUT);  //DIR
  // Motor P
  pinMode(12, OUTPUT);  //INA
  pinMode(14, OUTPUT);  //STP
  pinMode(27, OUTPUT);  //DIR
  digitalWrite(33, LOW);
  digitalWrite(12, LOW);
  digitalWrite(26, LOW);
  digitalWrite(27, HIGH);
  pid.setDeadzone(-5, 5);
}

int number;

void loop() {
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
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 2);
  rgx = Wire.read()<<8|Wire.read();
  groll = froll + grollcal + rgx*0.000031;
  froll = groll*0.997 + aroll*0.003;

  input = froll;
  pid.compute(micros());
  if((setpointMem == setpoint) && (output != 0))
  {
    if(output > 0)
    {
      setpoint += 0.007;
      setpointMem = setpoint;
    }
    else
    {
      setpoint -= 0.007;
      setpointMem = setpoint;
    }
  }
  if (output >= 0) {
    digitalWrite(26, LOW);
    digitalWrite(27, HIGH);
    number = numFinder(output);
    LMotor = number;
    RMotor = number;
  } else {
    digitalWrite(26, HIGH);
    digitalWrite(27, LOW);
    number = numFinder(-1 * output);
    LMotor = number;
    RMotor = number;
  }

  if(input >  15 || input < -15)
  {
    digitalWrite(33, HIGH);
    digitalWrite(12, HIGH);
  }

  while (ltime > micros());
  ltime = micros() + 4000;
}

void setup1() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  SerialBT.begin(device_name);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    SerialBT.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }
  ArduinoOTA.setHostname("SelfBalancingRobot");
  ArduinoOTA.setPasswordHash("ee338d1db6a6ed2564dc623d8f11bf8b");
  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH) {
        type = "sketch";
      } else {  // U_SPIFFS
        type = "filesystem";
      }
      SerialBT.println("Start updating " + type);
    })
    .onEnd([]() {
      SerialBT.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      SerialBT.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      SerialBT.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) {
        SerialBT.println("Auth Failed");
      } else if (error == OTA_BEGIN_ERROR) {
        SerialBT.println("Begin Failed");
      } else if (error == OTA_CONNECT_ERROR) {
        SerialBT.println("Connect Failed");
      } else if (error == OTA_RECEIVE_ERROR) {
        SerialBT.println("Receive Failed");
      } else if (error == OTA_END_ERROR) {
        SerialBT.println("End Failed");
      }
    });
  ArduinoOTA.begin();
}

char byt;
double KpBT = 0, KiBT = 0, KdBT = 0;
String val;
double remoteTime = 0;

void loop1() {
  ArduinoOTA.handle();
  if(SerialBT.available()) {
    byt = SerialBT.read();
    if(byt == 'P') {
      val = SerialBT.readStringUntil(' ');
      KpBT = val.toDouble();
    }
    if(byt == 'I') {
      val = SerialBT.readStringUntil(' ');
      KiBT = val.toDouble();
    }
    if(byt == 'D') {
      val = SerialBT.readStringUntil('\n');
      KdBT = val.toDouble();
      pid.setTunings(KpBT, KiBT, KdBT);
      SerialBT.print(KpBT);
      SerialBT.print(" ");
      SerialBT.print(KiBT);
      SerialBT.print(" ");
      SerialBT.println(KdBT);
    }
    if(byt == 'F')
    {
      setpoint += 0.7;
    }
    if(byt == 'B')
    {
      setpoint += -0.7;
    }
    if(byt == 'S')
    {
      setpoint = setpointMem;
    }
  }
  SerialBT.print(input);
  SerialBT.print(" ");
  SerialBT.print(setpoint);
  SerialBT.print(" ");
  SerialBT.println(output);
  // Serial.print(input);
  // Serial.print(" ");
  // Serial.print(setpoint);
  // Serial.print(" ");
  // Serial.println(output);
  vTaskDelay(1);
}
