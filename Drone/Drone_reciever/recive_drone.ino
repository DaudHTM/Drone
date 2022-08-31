#include <esp_now.h>
#include <WiFi.h>
#include<ESP32Servo.h>
#include "Wire.h"
#include<MPU6050_light.h>
#include <Arduino.h>
  
MPU6050 mpu(Wire);
float gyroX;
float gyroY;
#define tlPin 12
#define trPin 13
#define blPin 32
#define brPin 33

Servo tl;
Servo tr;
Servo br;
Servo bl;

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
  tl.write(70);
  tr.write(70);
  bl.write(70);
  br.write(70);
  delay(4000);
  tl.write(30);
  tr.write(0);
  bl.write(0);
  br.write(0);
  delay(2000);
  
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

  //Check Signal lost.
  unsigned long now = millis();
  if ( now - lastRecvTime > SIGNAL_TIMEOUT ) 
  {
    Serial.println("No Signaaal");  
    tl.write(50);
    bl.write(0);
    br.write(0);
    tr.write(0);
  }
   tl.write(map(myData.b,0, 4095,0,70));
   tr.write(map(myData.b,0, 4095,0,70));
   br.write(map(myData.b,0, 4095,0,70));
   bl.write(map(myData.b,0, 4095,0,70));

  gyroX=mpu.getAngleX();
  Serial.print(gyroX);
  Serial.print(" : ");
  gyroY=mpu.getAngleY();
  Serial.println(gyroY);
  delay(50);
  
}
