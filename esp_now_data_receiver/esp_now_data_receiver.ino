#include <esp_now.h>
#include <WiFi.h>

// Structure example to receive data
// Must match the sender's structure to ensure correct data interpretation
// Maximum size allowed for ESP-NOW messages is 250 bytes
typedef struct struct_message {
  float analog_1;  
  float analog_2; 
  float analog_3;  
  float analog_4;  
  float accx;      
  float accy;     
  float accz;      
  float gyrox;    
  float gyroy;     
  float gyroz;     
} struct_message;

// Create a struct_message instance called myData to hold incoming data
struct_message myData;

// Callback function that is executed when data is received
// The function copies the received data into the myData structure and prints it to the Serial Monitor
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  // Copy the received data into the myData structure
  memcpy(&myData, incomingData, sizeof(myData));

  // Create a formatted string with all the received data
  String dataStr = "Analog 1: " + String(myData.analog_1) + " " +
                   "Analog 2: " + String(myData.analog_2) + " " +
                   "Analog 3: " + String(myData.analog_3) + " " +
                   "Analog 4: " + String(myData.analog_4) + " " +
                   "AX: " + String(myData.accx) + " " +
                   "AY: " + String(myData.accy) + " " +
                   "AZ: " + String(myData.accz) + " " +
                   "GX: " + String(myData.gyrox) + " " +
                   "GY: " + String(myData.gyroy) + " " +
                   "GZ: " + String(myData.gyroz);
  
  // Print the received data to the Serial Monitor
  Serial.println(dataStr);
}

void setup() {
  // Initialize Serial Monitor for debugging
  Serial.begin(115200);

  // Set the device to Wi-Fi Station mode
  WiFi.mode(WIFI_STA);

  // Initialize ESP-NOW protocol
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register the callback function to be called when data is received
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));

  // Short delay to ensure setup is complete
  delay(500);
}

void loop() {
  // The loop function is intentionally left empty
  // All operations are handled by the callback function when data is received
}
