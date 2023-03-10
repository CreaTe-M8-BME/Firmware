
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

//BLE server name
#define bleServerName "BME_TEST_ESP32"

// Timer variables
unsigned long lastTime = 0;
unsigned long timerDelay = 10000;

bool deviceConnected = false;

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/
#define SERVICE_UUID "60ee560d-fb4e-4df4-8d02-dfd0dee58303"

// Test Characteristic and Descriptor
BLECharacteristic bmeTestCharacteristics("00000000-0000-0000-0000-000000000000", BLECharacteristic::PROPERTY_NOTIFY);
BLECharacteristic bmeTest2Characteristics("00000000-0000-0000-0000-000000000001", BLECharacteristic::PROPERTY_NOTIFY);
BLECharacteristic bmeTest3Characteristics("00000000-0000-0000-0000-000000000002", BLECharacteristic::PROPERTY_WRITE);
BLEDescriptor bmeTestDescriptor(BLEUUID((uint16_t)0x2902), BLECharacteristic::PROPERTY_READ);


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
      
      Serial.println(rxValue.c_str());
   }
};

void setup() {
  // Start serial communication 
  Serial.begin(115200);

  // Create the BLE Device
  BLEDevice::init(bleServerName);

  // Create the BLE Server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *bmeService = pServer->createService(SERVICE_UUID);
  
  bmeTest3Characteristics.setCallbacks(new SettingsUpdatedCallback());

  // Add BLE Characteristics and BLE Descriptor
  bmeService->addCharacteristic(&bmeTestCharacteristics);
  bmeService->addCharacteristic(&bmeTest2Characteristics);
  bmeService->addCharacteristic(&bmeTest3Characteristics);
  
  bmeTestDescriptor.setValue("This is a very beautifull sensor");
  
  bmeTestCharacteristics.addDescriptor(&bmeTestDescriptor);
  
  // Start the service
  bmeService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pServer->getAdvertising()->start();
  Serial.println("Waiting a client connection to notify...");
}

void loop() {
  if (deviceConnected) {
    if ((millis() - lastTime) > timerDelay) {
      float val = random(128);
            
      //Notify client
      static char sendValue[6];
      dtostrf(val, 6, 2, sendValue);
      
      //Set Tests Characteristic value and notify connected client
      bmeTestCharacteristics.setValue(sendValue);
      bmeTestCharacteristics.notify();   

      dtostrf(random(128), 6, 2, sendValue);
      bmeTest2Characteristics.setValue(sendValue);
      bmeTest2Characteristics.notify();   
      
      Serial.print(" - test Val: ");
      Serial.println(val);
      
      lastTime = millis();
    }
  }
}
