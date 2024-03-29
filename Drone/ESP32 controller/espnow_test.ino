

#include <esp_now.h>
#include <WiFi.h>






uint8_t broadcastAddress[] = {0x34, 0x94, 0x54, 0x25, 0x0C, 0x54};

// Define variables to store BME280 readings to be sent
float x;
float y;


// Define variables to store incoming readings
float Ax;
float Ay;


String success;

//Structure example to send data
//Must match the receiver structure
typedef struct struct_message {
    float x;
    float y;
    
} struct_message;

struct_message BME280Readings;

struct_message incomingReadings;

esp_now_peer_info_t peerInfo;

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status ==0){
    success = "Delivery Success :)";
  }
  else{
    success = "Delivery Fail :(";
  }
}

// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));

  Ax = incomingReadings.x;
  Ay = incomingReadings.y;
 
}
 
void setup() {

  Serial.begin(115200);


 
 
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

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
}
 
void loop() {

 
  // Set values to send
   BME280Readings.x = float(analogRead(33));
  BME280Readings.y = float(analogRead(32));



  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &BME280Readings, sizeof(BME280Readings));
   
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }



  
  // Display Readings in Serial Monitor
  Serial.println("INCOMING READINGS");
  Serial.print("Temperature: ");
  Serial.print(incomingReadings.x);
  Serial.println(" ºC");
  Serial.print("Humidity: ");
  Serial.print(incomingReadings.y);
    Serial.print("Temperature: ");
  Serial.print(BME280Readings.x);
  Serial.println(" ºC");
  Serial.print("Humidity: ");
  Serial.print(BME280Readings.y);
  Serial.println(" %");
  Serial.println();
  delay(100);
}
