// MPU-6050 Wireless Bluetooth Module
// By Jonathan Matarazzi
// May 5 2022
#include<Wire.h>

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#include "Esp.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
// #include "driver\rtc_io.h"

// BLE server name
#define BLE_NAME_PREFIX "BME_IMU_"
#define SERVICE_UUID "0ddf5c1d-d269-4b17-bd7f-33a8658f0b89"

#define BLUE_PIN 12
#define RED_PIN 13
#define MPU_POWER_PIN 19
#define PAIR_PIN 26
#define POWER_OFF_PIN 32
#define POWER_OFF_PIN_GPIO GPIO_NUM_32
#define WIRED_MODE_PIN 33
#define WIRELESS_MODE_PIN 25

const uint16_t MIN_SAMPLING_DELAY = 5;
const uint16_t MAX_SAMPLING_DELAY = 2000;

// Timer variables
unsigned long lastTime = 0;
unsigned long samplingDelay = 1000;

bool deviceConnected = false;

// Test Characteristic and Descriptor
BLECharacteristic imuCharacteristics("64b83770-6b12-4a54-b31a-e007306132bd", BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_WRITE);
BLEDescriptor imuDescriptionDescriptor(BLEUUID((uint16_t)0x2901));
BLE2902 *imu2902 = new BLE2902();
BLECharacteristic sampleRateCharacteristics("3003aac7-d843-4e55-9d89-3f93020cc9ee", BLECharacteristic::PROPERTY_WRITE);
BLEDescriptor sampleRateDescriptionDescriptor(BLEUUID((uint16_t)0x2901));

BLEServer *pServer;

const int MPU_addr = 0x68; // I2C address of the MPU-6050
byte output[12];

bool transmitMode;

TaskHandle_t readIMU, clientHandler, pairingTask;


// Setup callbacks onConnect and onDisconnect
class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    Serial.println("Client connected");
  };
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    Serial.println("Client disconnected");
    pServer->getAdvertising()->start();
  }
};

union ByteArrayToInt {
  byte array[2];
  uint16_t integer;
};

// Setup data recieved callback
class SettingsUpdatedCallback: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      uint8_t* rxValue = pCharacteristic->getData();

      ByteArrayToInt converter;
      converter.array[0] = rxValue[1];
      converter.array[1] = rxValue[0];
      Serial.print("Sampling delay: ");
      Serial.println(converter.integer);
      
      // limit the sampling delay to the max and min value
      samplingDelay = std::max(std::min(converter.integer, MAX_SAMPLING_DELAY), MIN_SAMPLING_DELAY);
   }
};

void setup() {
  Serial.begin(115200);

  //Set pinmodes for slider en button
  pinMode(BLUE_PIN, OUTPUT);
  pinMode(RED_PIN, OUTPUT);
  pinMode(MPU_POWER_PIN, OUTPUT);
  pinMode(PAIR_PIN, INPUT_PULLUP);
  pinMode(POWER_OFF_PIN, INPUT_PULLUP);
  pinMode(WIRED_MODE_PIN, INPUT_PULLUP);
  pinMode(WIRELESS_MODE_PIN, INPUT_PULLUP);

  digitalWrite(BLUE_PIN, HIGH);
  digitalWrite(RED_PIN, HIGH);
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
    0
  );

  xTaskCreatePinnedToCore(
    clientHandlerCode,
    "ClientHandler",
    10000,
    NULL,
    1,
    &clientHandler,
    1
  );
}

void readIMUCode( void * parameter) {
  for (;;) {
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr, 14, true); // request a total of 14 registers
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

void clientHandlerCode ( void * parameters) {
  for (;;) {
    if (transmitMode) {
      unsigned long curTime = millis();
      if (deviceConnected && (curTime - lastTime) >= samplingDelay) {
        // Set Characteristic value and notify connected client
        imuCharacteristics.setValue(output, 12);
        
        imuCharacteristics.notify();
        lastTime = curTime;
        Serial.print("Sent data: ");
        for (int i = 0; i < 6; i++) {
          Serial.print((int)(output[i*2] | output[i*2+1] << 8));
          Serial.print(", ");
        }
        Serial.println();
      }
    } else {
      // Wired mode
      if (Serial.available() > 0) {
        Serial.read();
        Serial.read();
        for (int i = 0; i < 6; i++) {
          int16_t value = (output[i*2] << 8) + output[i*2+1];
          Serial.print(i);
          Serial.print(" value: ");
          Serial.println(value);
        }
      }
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
  digitalWrite(BLUE_PIN, HIGH);
  digitalWrite(RED_PIN, HIGH);
  esp_sleep_enable_ext0_wakeup(POWER_OFF_PIN_GPIO, 1);
  esp_deep_sleep_start();
}


void loop() {
  // Check de state of de button
  if (digitalRead(POWER_OFF_PIN) == LOW) {
    startSleep();
  } else if (digitalRead(WIRED_MODE_PIN) == LOW) {
    transmitMode = false;

    // turn on the red light
    digitalWrite(BLUE_PIN, HIGH);
    digitalWrite(RED_PIN, LOW);
  } else if (digitalRead(WIRELESS_MODE_PIN) == LOW) {
    transmitMode = true;

    // turn on the blue light
    digitalWrite(RED_PIN, HIGH);
    digitalWrite(BLUE_PIN, LOW);
  }
}

void SetupBLE() {
  Serial.println("Setting up BLE..");
  // Create the BLE Device
  // Serial.println(getBluetoothAddress());
  BLEDevice::init(BLE_NAME_PREFIX+getBluetoothAddress());

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
  sampleRateDescriptionDescriptor.setValue("int16_t delay_val");
  sampleRateCharacteristics.addDescriptor(&sampleRateDescriptionDescriptor);
  
  // Start the service
  dataService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pServer->getAdvertising()->start();

  Serial.println("Waiting on client...");
}

std::string getBluetoothAddress() {
  btStart();
  esp_bluedroid_init();
  esp_bluedroid_enable();
  const uint8_t* address = esp_bt_dev_get_address();
  std::string addrString = std::string();
  for (int i = 0; i < 2; i++) {
    char str[3];
    sprintf(str, "%02X", (int)address[i+4]);
    addrString = std::string(addrString + str);
  }
  esp_bluedroid_disable();
  esp_bluedroid_deinit();
  btStop();
  return addrString;
}
