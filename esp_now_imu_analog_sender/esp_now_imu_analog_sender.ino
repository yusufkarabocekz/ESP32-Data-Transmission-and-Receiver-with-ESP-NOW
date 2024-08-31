/*
 * This program uses ESP-NOW to send data from an ESP32 to a receiver.
 * The data includes readings from an MPU9255 IMU (Inertial Measurement Unit)
 * and four analog sensors connected to the ESP32.
 * The program creates two tasks: 
 * - Task1 reads sensor data and prepares it for transmission.
 * - Task2 handles the data transmission using the ESP-NOW protocol.
 * 
 * The receiver MAC address should be updated to match your specific receiver.
 */

#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <MPU9255.h>

TaskHandle_t Task1; // Task for reading sensor data
TaskHandle_t Task2; // Task for sending data

#define g 9.81 // Acceleration due to gravity (1g ~ 9.81 m/s^2)

// Receiver's MAC Address (Replace this with your receiver's MAC)
uint8_t broadcastAddress[] = {0x3C, 0x71, 0xBF, 0xAA, 0xF3, 0xA0};

MPU9255 mpu;

const int analogPin_1 = 27; // Define analog input pins
const int analogPin_2 = 14;
const int analogPin_3 = 13;
const int analogPin_4 = 12;

int analogValue_1 = 0; // Variables to store analog readings
int analogValue_2 = 0;
int analogValue_3 = 0;
int analogValue_4 = 0;

// Struct to hold the data that will be sent
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

struct_message myData; // Create an instance of the struct to hold the data

esp_now_peer_info_t peerInfo; // ESP-NOW peer information

// Callback function that is executed when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

/*
 * Function to convert raw acceleration data from the MPU9255 into m/s^2
 */
double process_acceleration(int input, scales sensor_scale ) {
  double output = 1;
  if (sensor_scale == scale_2g)
    output = input / 16384.0 * g;
  else if (sensor_scale == scale_4g)
    output = input / 8192.0 * g;
  else if (sensor_scale == scale_8g)
    output = input / 4096.0 * g;
  else if (sensor_scale == scale_16g)
    output = input / 2048.0 * g;
  return output;
}

/*
 * Function to convert raw gyroscopic data from the MPU9255 into degrees per second
 */
double process_angular_velocity(int16_t input, scales sensor_scale ) {
  if (sensor_scale == scale_250dps)
    return input / 131.0;
  else if (sensor_scale == scale_500dps)
    return input / 65.5;
  else if (sensor_scale == scale_1000dps)
    return input / 32.8;
  else if (sensor_scale == scale_2000dps)
    return input / 16.4;
  return 0;
}

void setup() {
  // Initialize Serial Monitor for debugging
  Serial.begin(115200);
  Wire.begin();
  mpu.init(); // Initialize the MPU9255
  WiFi.mode(WIFI_STA); // Set ESP32 as a Wi-Fi Station

  // Initialize ESP-NOW protocol
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_send_cb(OnDataSent); // Register callback for sending data

  // Set the peer information for the receiver
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add the receiver as a peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  // Create Task1 to read sensor data, pinned to core 0
  xTaskCreatePinnedToCore(
    Task1code,   /* Task function. */
    "Task1",     /* Name of task. */
    10000,       /* Stack size of task */
    NULL,        /* Parameter of the task */
    1,           /* Priority of the task */
    &Task1,      /* Task handle to keep track of created task */
    0);          /* Pin task to core 0 */
  delay(500);

  // Create Task2 to send data, pinned to core 1
  xTaskCreatePinnedToCore(
    Task2code,   /* Task function. */
    "Task2",     /* Name of task. */
    10000,       /* Stack size of task */
    NULL,        /* Parameter of the task */
    1,           /* Priority of the task */
    &Task2,      /* Task handle to keep track of created task */
    1);          /* Pin task to core 1 */
  delay(500);
}

// Task1: Reads sensor data and prepares it for transmission
void Task1code( void * pvParameters ) {
  for (;;)
  {
    mpu.read_acc(); // Read accelerometer data
    mpu.read_gyro(); // Read gyroscope data

    // Read analog sensor values
    analogValue_1 = analogRead(analogPin_1);
    analogValue_2 = analogRead(analogPin_2);
    analogValue_3 = analogRead(analogPin_3);
    analogValue_4 = analogRead(analogPin_4);

    // Store the analog values in the struct
    myData.analog_1 = analogValue_1;
    myData.analog_2 = analogValue_2;
    myData.analog_3 = analogValue_3;
    myData.analog_4 = analogValue_4;

    // Process and store the IMU data in the struct
    myData.accx = process_acceleration(mpu.ax, scale_2g);
    myData.accy = process_acceleration(mpu.ay, scale_2g);
    myData.accz = process_acceleration(mpu.az, scale_2g);

    myData.gyrox = process_angular_velocity(mpu.gx, scale_250dps);
    myData.gyroy = process_angular_velocity(mpu.gy, scale_250dps);
    myData.gyroz = process_angular_velocity(mpu.gz, scale_250dps);

    // Optionally, you can print the sensor data for debugging
    /*
    Serial.print("Analog 1: ");
    Serial.println(analogValue_1);
    Serial.print("Analog 2: ");
    Serial.println(analogValue_2);
    Serial.print("Analog 3: ");
    Serial.println(analogValue_3);
    Serial.print("Analog 4: ");
    Serial.println(analogValue_4);

    Serial.print("AX: "); Serial.println(process_acceleration(mpu.ax, scale_2g));
    Serial.print("AY: "); Serial.println(process_acceleration(mpu.ay, scale_2g));
    Serial.print("AZ: "); Serial.println(process_acceleration(mpu.az, scale_2g));

    Serial.print("GX: "); Serial.println(process_angular_velocity(mpu.gx, scale_250dps));
    Serial.print("GY: "); Serial.println(process_angular_velocity(mpu.gy, scale_250dps));
    Serial.print("GZ: "); Serial.println(process_angular_velocity(mpu.gz, scale_250dps));
    */
  }
}

// Task2: Sends the sensor data via ESP-NOW
void Task2code( void * pvParameters ) {
  for (;;) {
    // Send the data stored in the struct via ESP-NOW
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));

    // Optionally, you can print the send status for debugging
    /*
    if (result == ESP_OK) {
      Serial.println("Sent with success");
    } else {
      Serial.println("Error sending the data");
    }
    delay(10); 
    */
  }
}

// Main loop (not used since tasks handle everything)
void loop() {
  // The main loop is left empty as the tasks are managing all operations.
}
