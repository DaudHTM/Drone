
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
#define blPin 33
#define brPin 32

Servo tl;
Servo tr;
Servo br;
Servo bl;
bool off=false;

float tlVal=0;
float trVal=0;
float brVal=0;
float blVal=0;

float tlValT=78;
float trValT=78;
float brValT=78;
float blValT=78;

float rollTime, timeX, rollTimePrev;

float pitchTime, pitchX, pitchTimePrev;


float pRoll;
float iRoll;
float dRoll;

float pRollG=0.9;
float iRollG=0.000;
float dRollG=0.15;

int maxRoll;

float pPitch;
float iPitch;
float dPitch;

float pPitchG=0.9;
float iPitchG= 0.000;
float dPitchG=0.15;
int maxPitch;

float targetX=0.0;
float targetY=0.0;

float rollError;
float pitchError;
float rollErrorP;
float pitchErrorP;

float pidX;
float pidY;

#define SIGNAL_TIMEOUT 500  // This is signal timeout in milli seconds. We will reset the data if no signal

unsigned long lastRecvTime = 0;
String projectTitle = "Beehive Counter";
String projectDesc =
    "ESP32 for YouTube demonstration! <p>You can write text here or <b>html</b>.</p> \
  <p>Note: Try this on another ESP32 device before demo. And other important info. Keep it concise!</p>";
const char* SSID="Verizon_36XGZN";
const char* PASSWORD="********";
#include "identification.h"

uint8_t broadcastAddress[] = {0x34, 0x94, 0x54, 0x25, 0x20, 0xC0};

// Define variables to store BME280 readings to be sent
float x;
float y;


// Define variables to store incoming readings
float Ax;
float Ay;


// Variable to store if sending data was successful
String success;

//Structure example to send data
//Must match the receiver structure
typedef struct struct_message {
    float x;
    float y;
    
} struct_message;

// Create a struct_message called BME280Readings to hold sensor readings
struct_message angle;

// Create a struct_message to hold incoming sensor readings
struct_message incomingReadings;

esp_now_peer_info_t peerInfo;

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status ==0){
    success = "Delivery Success :)";
    off=false;
  }
  else{
    success = "Delivery Fail :(";
      tl.write(0);
      tr.write(0);
      bl.write(0);
      br.write(0);
      off=true;
    
      
  }
}

// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));
  Serial.print("Bytes received: ");
  Serial.println(len);
  Ax = incomingReadings.x;
  Ay = incomingReadings.y;
 
}

void setup() 
{
  Serial.begin(115200);
 
  WiFi.mode(WIFI_STA);
  
  Wire.begin();
  mpu.begin();
  
  tr.attach(trPin,1000,2000);
  tl.attach(tlPin,1000,2000);
  br.attach(brPin,1000,2000);
  bl.attach(blPin,1000,2000);
  pinMode(2,OUTPUT);
  tl.write(30);
  tr.write(30);
  bl.write(30);
  br.write(30);
  delay(3000);
  tl.write(0);
  tr.write(0);
  bl.write(0);
  br.write(0);
  delay(2000);
  
  // Init ESP-NOW



  
  otaInit();

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);

 
  mpu.calcOffsets();
  mpu.setFilterGyroCoef(.99);
  delay(500);

  timeX=millis();
}
 


void loop()
{
  mpu.update();
  rollTimePrev=timeX;
  pitchTimePrev=timeX;
  timeX=millis();
  rollTime=(timeX-rollTimePrev)/1000;
  pitchTime=(timeX-pitchTimePrev)/1000;








  
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &angle, sizeof(angle));
  
  Serial.print(gyroX);
  Serial.print(" : ");
  Serial.println(gyroY);

     
  if (result == ESP_OK) {
    Serial.println("S  ent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
  
  
  Serial.println("INCOMING READINGS");
  Serial.print("Temperature: ");
  Serial.print(incomingReadings.x);
  Serial.println(" ºC");
  Serial.print("Humidity: ");
  Serial.print(incomingReadings.y);
  Serial.println(" %");
  Serial.println();

  trValT=map(incomingReadings.x,0,4095,0,126);
  tlValT=trValT;
  brValT=trValT;
  blValT=trValT;
  
  angle.y = (mpu.getAngleX());
  angle.x =(1*(mpu.getAngleY()));



  rollError=angle.x-targetX;

  pRoll=pRollG*rollError;
  
  

    iRoll=iRoll+(iRollG*rollError);
  

  dRoll=dRollG*((rollError-rollErrorP)/rollTime);

  pidX=pRoll+iRoll+dRoll;
  
  trVal=trValT+pidX;
  brVal=brValT+pidX;
  tlVal=tlValT-pidX;
  blVal=blValT-pidX;




  pitchError=angle.y-targetY;

  pPitch=pPitchG*pitchError;
  
  

    iPitch=iPitch+(iPitchG*pitchError);


  dPitch=dPitchG*((pitchError-pitchErrorP)/pitchTime);

  pidY=pPitch+iPitch+dPitch;
  
  trVal=trVal-pidY;
  brVal=brVal+pidY;
  tlVal=tlVal-pidY;
  blVal=blVal+pidY;

  trVal=min(trVal,float(160));
  brVal=min(brVal,float(160));
  tlVal=min(tlVal,float(160));
  blVal=min(blVal,float(160));
  
  if(off!=true){
    
    tl.write(tlVal-4);
    bl.write(blVal-4);
    br.write(brVal+5);
    tr.write(trVal+4);
    
  }
  else{
    
    tl.write(0);
    bl.write(0);
    br.write(0);
    tr.write(0);
  }

  rollErrorP=rollError;
  pitchErrorP=pitchError;


  
}
