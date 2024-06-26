// MPU-6050 Wireless Bluetooth Module
// By Jonathan Matarazzi
// May 5 2022
// Updated by Frank Bosman
// April 3 2024
#define VERSION "1.0.3"

// Uncomment the line below to turn on debugging info over Serial communication
// #define DEBUG_MODE

#include <Wire.h>
#include <math.h>

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#include "Esp.h"
#include "esp_timer.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
// #include "driver\rtc_io.h"

// BLE server name
#define BLE_NAME_PREFIX "BME_IMU_"
#define SERVICE_UUID "0ddf5c1d-d269-4b17-bd7f-33a8658f0b89"
#define IMU_UUID "64b83770-6b12-4a54-b31a-e007306132bd"
#define SAMPLE_RATE_UUID "3003aac7-d843-4e55-9d89-3f93020cc9ee"
#define VERSION_UUID "2980b86f-dacb-43b9-847c-30c586224943"

#define LED_R 27
#define LED_G 13
#define LED_B 12
#define MPU_POWER_PIN 19
#define POWER_OFF_PIN 32
#define POWER_OFF_PIN_GPIO GPIO_NUM_32
#define POWER_ON_PIN 33

// PWM properties
#define PWM_FREQUENCY 5000
#define PWN_CHANNEL_BLUE 0
#define PWN_CHANNEL_GREEN 1
#define PWM_RESOLUTION 8
#define LED_BRIGHTNESS 50

#define MIN_SAMPLING_FREQUENCY 1
#define MAX_SAMPLING_FREQUENCY 200

#define TIMER_PRECISION 1000000

#define DISCO_INTERVAL 100
#define NOT_CONNECTED_INTERVAL 500

// Device mode variables
int discoState = 0;
unsigned long discoPrevTime = 0;
unsigned long notConnectedPrevTime = 0;
bool notConnectedLEDActive = false;

// Timer variables
hw_timer_t *blTimer = timerBegin(0, 1000000, true);
unsigned long lastTime = 0;
uint16_t frequency = 100;
unsigned long samplingDelay = 10;

bool deviceConnected = false;

// Test Characteristic and Descriptor
BLECharacteristic imuCharacteristics(IMU_UUID, BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_WRITE);
BLEDescriptor imuDescriptionDescriptor(BLEUUID((uint16_t)0x2901));
BLE2902 *imu2902 = new BLE2902();
BLECharacteristic sampleRateCharacteristics(SAMPLE_RATE_UUID, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_READ);
BLEDescriptor sampleRateDescriptionDescriptor(BLEUUID((uint16_t)0x2901));
BLECharacteristic versionCharacteristic(VERSION_UUID, BLECharacteristic::PROPERTY_READ);
BLEDescriptor versionDescriptionDescriptor(BLEUUID((uint16_t)0x2901));

BLEServer *pServer;

const int MPU_addr = 0x68;  // I2C address of the MPU-6050
byte output[12];

TaskHandle_t readIMU, clientHandler, pairingTask;

// Setup callbacks onConnect and onDisconnect
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    deviceConnected = true;
    pServer->getAdvertising()->stop();
    #ifdef DEBUG_MODE
    Serial.println("Client connected");
    #endif
  };
  void onDisconnect(BLEServer *pServer) {
    deviceConnected = false;
    pServer->getAdvertising()->start();
    #ifdef DEBUG_MODE
    Serial.println("Client disconnected");
    #endif
  }
};

// Setup data recieved callback
class SettingsUpdatedCallback : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    uint8_t *rxValue = pCharacteristic->getData();
    uint16_t value = (rxValue[1] << 8) | rxValue[0];

    // limit the sampling delay to the max and min value
    int reqFrequency = std::max(std::min(value, (uint16_t)MAX_SAMPLING_FREQUENCY), (uint16_t)MIN_SAMPLING_FREQUENCY);
    samplingDelay = round((float)TIMER_PRECISION / (float)reqFrequency);
    frequency = TIMER_PRECISION / samplingDelay;
    #ifdef DEBUG_MODE
    Serial.print("Sampling delay:\t");
    Serial.println(samplingDelay);
    Serial.print("Sampling frequency:\t");
    Serial.println(frequency);
    #endif
    pCharacteristic->setValue(frequency);
  }
};

void setup() {
  #ifdef DEBUG_MODE
  Serial.begin(115200);
  #endif

  //Set pinmodes for slider en button
  pinMode(LED_R, OUTPUT);
  pinMode(MPU_POWER_PIN, OUTPUT);
  pinMode(POWER_OFF_PIN, INPUT_PULLUP);
  pinMode(POWER_ON_PIN, INPUT_PULLUP);

  // Setup PWM for green and blue LEDs
  ledcSetup(PWN_CHANNEL_GREEN, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(PWN_CHANNEL_BLUE, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(LED_G, PWN_CHANNEL_GREEN);
  ledcAttachPin(LED_B, PWN_CHANNEL_BLUE);

  digitalWrite(LED_R, LOW);
  ledcWrite(PWN_CHANNEL_GREEN, 0);
  ledcWrite(PWN_CHANNEL_BLUE, 0);
  digitalWrite(MPU_POWER_PIN, HIGH);

  delay(100);

  //Wake up MPU-6050 and set accelerometer and gyroscope to correct ranges
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(false);
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1B);
  Wire.write(0b00011000);
  Wire.endTransmission(false);
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1C);
  Wire.write(0b00011000);
  Wire.endTransmission(false);

  // Setup Bluetooth
  SetupBLE();

  //Create repeating tasks
  xTaskCreatePinnedToCore(
    readIMUCode,
    "ReadIMU",
    10000,
    NULL,
    0,
    &readIMU,
    0);

  xTaskCreatePinnedToCore(
    clientHandlerCode,
    "ClientHandler",
    10000,
    NULL,
    1,
    &clientHandler,
    1);
}

void readIMUCode(void *parameter) {
  for (;;) {
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr, 14, true);  // request a total of 14 registers
    for (int i = 0; i < 14; i++) {
      byte input = Wire.read();
      if (i < 6) {
        output[i] = input;
      } else if (i > 7) {
        output[i - 2] = input;
      }
    }
  }
}

void clientHandlerCode(void *parameters) {
  for (;;) {
    unsigned long curTime = esp_timer_get_time();
    if ((curTime - lastTime) >= samplingDelay) {
      if (deviceConnected) {
        // Set Characteristic value and notify connected client
        imuCharacteristics.setValue(output, 12);

        imuCharacteristics.notify();
      }
      lastTime += samplingDelay;
    }
  }
}

/**
 * Puts the ESP into a sleep state
 */
void startSleep() {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0b01001000);
  Wire.endTransmission(false);
  digitalWrite(MPU_POWER_PIN, LOW);
  digitalWrite(LED_R, LOW);
  ledcWrite(PWN_CHANNEL_GREEN, 0);
  ledcWrite(PWN_CHANNEL_BLUE, 0);
  esp_sleep_enable_ext0_wakeup(POWER_OFF_PIN_GPIO, 1);
  esp_deep_sleep_start();
}


void loop() {
  // Check de state of de button
  bool powerOnPinState = digitalRead(POWER_ON_PIN) == LOW;
  bool powerOffPinState = digitalRead(POWER_OFF_PIN) == LOW;

  // Switch between the three different modes, Disco, off/sleep and active
  if (!powerOnPinState && !powerOffPinState) {  // Disco mode
    unsigned long curTime = millis();
    if (curTime - discoPrevTime >= DISCO_INTERVAL) {
      discoPrevTime = curTime;
      discoState = (discoState + 1) % 3;
      digitalWrite(LED_R, discoState == 0 ? HIGH : LOW);
      ledcWrite(PWN_CHANNEL_GREEN, discoState == 1 ? 255 : 0);
      ledcWrite(PWN_CHANNEL_BLUE, discoState == 2 ? 255 : 0);
    }
  } else if (powerOffPinState) {  
    // Sleep mode, turn device off
    startSleep();
  } else if (powerOnPinState) {
    // Active mode, show a indicator light
    digitalWrite(LED_R, LOW);
    if (deviceConnected) {
      // Device connected, show a green LED
      ledcWrite(PWN_CHANNEL_GREEN, LED_BRIGHTNESS);
      ledcWrite(PWN_CHANNEL_BLUE, 0);
    } else {
      // No device connected, show a blinking blue LED
      unsigned long curTime = millis();
      if (curTime - notConnectedPrevTime >= NOT_CONNECTED_INTERVAL) {
        notConnectedPrevTime = curTime;
        notConnectedLEDActive = !notConnectedLEDActive;
      }
      ledcWrite(PWN_CHANNEL_GREEN, 0);
      ledcWrite(PWN_CHANNEL_BLUE, notConnectedLEDActive ? LED_BRIGHTNESS : 0);
    }
  }
}

void SetupBLE() {
  #ifdef DEBUG_MODE
  Serial.println("Setting up BLE..");
  #endif
  // Create the BLE Device
  BLEDevice::init(BLE_NAME_PREFIX + getBluetoothAddress());

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *dataService = pServer->createService(SERVICE_UUID);

  sampleRateCharacteristics.setCallbacks(new SettingsUpdatedCallback());

  // Add BLE Characteristics and BLE Descriptor
  dataService->addCharacteristic(&imuCharacteristics);
  imuDescriptionDescriptor.setValue("uint16_t acc_x, uint16_t acc_y, uint16_t acc_z, uint16_t gyr_x, uint16_t gyr_y, uint16_t gyr_z");
  imuCharacteristics.addDescriptor(&imuDescriptionDescriptor);
  imuCharacteristics.addDescriptor(imu2902);
  dataService->addCharacteristic(&sampleRateCharacteristics);
  sampleRateDescriptionDescriptor.setValue("int16_t sample_frequency");
  sampleRateCharacteristics.addDescriptor(&sampleRateDescriptionDescriptor);
  dataService->addCharacteristic(&versionCharacteristic);
  versionDescriptionDescriptor.setValue("Firmware version");
  versionCharacteristic.addDescriptor(&versionDescriptionDescriptor);
  versionCharacteristic.setValue(VERSION);

  // Start the service
  dataService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pServer->getAdvertising()->start();

  #ifdef DEBUG_MODE
  Serial.println("Waiting on client...");
  #endif
}

std::string getBluetoothAddress() {
  btStart();
  esp_bluedroid_init();
  esp_bluedroid_enable();
  const uint8_t *address = esp_bt_dev_get_address();
  std::string addrString = std::string();
  for (int i = 0; i < 2; i++) {
    char str[3];
    sprintf(str, "%02X", (int)address[i + 4]);
    addrString = std::string(addrString + str);
  }
  esp_bluedroid_disable();
  esp_bluedroid_deinit();
  btStop();
  return addrString;
}
