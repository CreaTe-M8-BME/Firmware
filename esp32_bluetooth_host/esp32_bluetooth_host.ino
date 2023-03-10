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
#include "driver\rtc_io.h"

// BLE server name
#define bleServerName "BME_IMU_0BE6"
#define SERVICE_UUID "0ddf5c1d-d269-4b17-bd7f-33a8658f0b89"

#define BLUE_PIN 13
#define RED_PIN 12
#define MPU_POWER_PIN 19
#define PAIR_PIN 26
#define POWER_OFF_PIN 32
#define POWER_OFF_PIN_GPIO GPIO_NUM_32
#define WIRED_MODE_PIN 33
#define WIRELESS_MODE_PIN 25

// Timer variables
unsigned long lastTime = 0;
unsigned long samplingDelay = 5000;

bool deviceConnected = false;

// Test Characteristic and Descriptor
BLECharacteristic accelerometerCharacteristics("00000000-0000-0000-0000-000000000001", BLECharacteristic::PROPERTY_NOTIFY);
BLECharacteristic gyroscopeCharacteristics("00000000-0000-0000-0000-000000000002", BLECharacteristic::PROPERTY_NOTIFY);
BLECharacteristic sampleRateCharacteristics("00000000-0000-0000-0000-000000000003", BLECharacteristic::PROPERTY_WRITE);
BLEDescriptor dataDescriptor(BLEUUID((uint16_t)0x2902), BLECharacteristic::PROPERTY_READ);


const int MPU_addr = 0x68; // I2C address of the MPU-6050
byte output[12];

bool transmitMode;

TaskHandle_t readIMU, clientHandler, pairingTask;


// Setup callbacks onConnect and onDisconnect
class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
  };
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
  }
};

// Setup data recieved callback
class SettingsUpdatedCallback: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();
      
      Serial.println(rxValue.c_str()); //TODO FIX
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
      if (deviceConnected && (millis() - lastTime) > samplingDelay) {
        // char vals[24];
        // char sendValue[3];
        //  for (int i = 0; i < 6; i++) {
          
        //   Serial.print(i);
        //   Serial.print(" value: ");
        //   Serial.print(output[i]);
        //   Serial.print(" - ");
        //   sprintf(sendValue, "%3.03d", output[i]);
        //   Serial.println(sendValue);
        //   strcat(vals, sendValue);
        // }

        // Serial.print("Final: ");
        // Serial.println(vals);
        // Set Tests Characteristic value and notify connected client
        accelerometerCharacteristics.setValue(output, 12);
        
        accelerometerCharacteristics.notify();
        // strcpy(vals, "");
        lastTime = millis();
      }
    } else {
      // Wired mode
      if (Serial.available() > 0) {
        Serial.read();
        Serial.read();
        for (int i = 0; i < 6; i++) {
          uint16_t value = (output[i*2] << 8) + output[i*2+1];
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
    digitalWrite(BLUE_PIN, HIGH);
    digitalWrite(RED_PIN, LOW);
  } else if (digitalRead(WIRELESS_MODE_PIN) == LOW) {
    transmitMode = true;
    digitalWrite(RED_PIN, HIGH);
  }
}

void SetupBLE() {
  Serial.println("Setting up BLE..");
  // Create the BLE Device
  BLEDevice::init(bleServerName);

  // Create the BLE Server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *dataService = pServer->createService(SERVICE_UUID);
  
  sampleRateCharacteristics.setCallbacks(new SettingsUpdatedCallback());

  // Add BLE Characteristics and BLE Descriptor
  dataService->addCharacteristic(&accelerometerCharacteristics);
  dataService->addCharacteristic(&gyroscopeCharacteristics);
  dataService->addCharacteristic(&sampleRateCharacteristics);
  
  // Start the service
  dataService->start();
 

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pServer->getAdvertising()->start();

  Serial.println("Waiting on client...");
}
