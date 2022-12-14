                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             #include <esp_now.h>
#include <WiFi.h>
#include<ESP32Servo.h>
#include "Wire.h"
#include<MPU6050_light.h>
#include <Arduino.h>
int ons=1;
MPU6050 mpu(Wire);
float gyroX;
float gyroY;
#define tlPin 12
#define trPin 13
#define blPin 33
#define brPin 32

Servo tl;
Servo tr;
Servo br;
Servo bl;

float XelapsedTime, Xtime, XtimePrev;
float YelapsedTime, Ytime, YtimePrev;

float tlVal=70;
float trVal=70;
float blVal=70;
float brVal=70;

float Xtarget=0;
float Ytarget=0;

float Yerror;
float YerrorP;

float Xerror;
float XerrorP;

float Xpid_p=0;
float Xpid_i=0;
float Xpid_d=0;
float XPID=0;
/////////////////PID CONSTANTS/////////////////
double Xkp=3.55;//3.55
double Xki=0.005;//0.003
double Xkd=2.05;//2.05

float Ypid_p=0;
float Ypid_i=0;
float Ypid_d=0;
float YPID=0;
/////////////////PID CONSTANTS/////////////////
double Ykp=3.55;//3.55
double Yki=0.005;//0.003
double Ykd=2.05;//2.05



#define SIGNAL_TIMEOUT 1000  // This is signal timeout in milli seconds. We will reset the data if no signal

unsigned long lastRecvTime = 0;
String projectTitle = "Beehive Counter";
String projectDesc =
    "ESP32 for YouTube demonstration! <p>You can write text here or <b>html</b>.</p> \
  <p>Note: Try this on another ESP32 device before demo. And other important info. Keep it concise!</p>";
const char* SSID="Verizon_36XGZN";
const char* PASSWORD="drain7-haft-dos";
#include "identification.h"

typedef struct struct_message {
    
    int b;
    int c;
} struct_message;

// Create a struct_message called myData
struct_message myData;
// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.print("Bytes received: ");
  Serial.println(len);

  Serial.print("Int: ");
  Serial.println(myData.b);
  Serial.print("Float: ");
  Serial.println(myData.c);

  Serial.println();
  lastRecvTime = millis(); 
}
void setup() 
{
  Serial.begin(115200);
 
  WiFi.mode(WIFI_STA);
  mpu.begin();
  Wire.begin();

  tr.attach(trPin,1000,2000);
  tl.attach(tlPin,1000,2000);
  br.attach(brPin,1000,2000);
  bl.attach(blPin,1000,2000);
  pinMode(2,OUTPUT);
  tl.write(60);
  tr.write(60);
  bl.write(60);
  br.write(60);
  delay(1000);
  tl.write(0);
  tr.write(0);
  bl.write(0);
  br.write(0);
  delay(5000);
  
  // Init ESP-NOW

  if (esp_now_init() != ESP_OK) 
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  mpu.calcGyroOffsets();
  delay(500);
  mpu.setFilterGyroCoef(.97);
  mpu.setFilterAccCoef(.03);
  
  
  otaInit();
  esp_now_register_recv_cb(OnDataRecv);
}
 


void loop()
{
  mpu.update();

    XtimePrev = Xtime;  // the previous time is stored before the actual time read
    Xtime = millis();  // actual time read
    XelapsedTime = (Xtime - XtimePrev) / 1000; 

    YtimePrev = Ytime;  // the previous time is stored before the actual time read
    Ytime = millis();  // actual time read
    YelapsedTime = (Ytime - YtimePrev) / 1000; 

  //Check Signal lost.
  unsigned long now = millis();
  ons=1;
  if ( now - lastRecvTime > SIGNAL_TIMEOUT ) 
  {
    Serial.println("No Signaaal");  
    ons=0;
  }
  else{
    ons=1;
  }
   /*tlVal=((map(myData.b,0, 4095,0,80))+9);
   trVal=(map(myData.b,0, 4095,0,80));
   brVal=(map(myData.b,0, 4095,0,80));
   blVal=(map(myData.b,0, 4095,0,80));*/

  gyroX=mpu.getAngleX();
  Serial.print(gyroX);
  Serial.print(" : ");
  gyroY=mpu.getAngleY();
  Serial.println(gyroY);
  

  Xerror=gyroX-Xtarget;

  Xpid_p=Xkp*Xerror;

  tlVal=60-Xpid_p;
  blVal=60-Xpid_p;
  trVal=60+Xpid_p;
  brVal=60+Xpid_p;
  
  tlVal=min(float(75.0),tlVal);
  trVal=min(float(75.0),trVal);
  blVal=min(float(75.0),blVal);
  brVal=min(float(75.0),brVal);

  tl.write(tlVal*ons);
  bl.write(blVal*ons);
  tr.write(trVal*ons);
  br.write(brVal*ons);
}
