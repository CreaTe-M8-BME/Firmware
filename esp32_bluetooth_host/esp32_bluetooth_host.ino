// MPU-6050 Wireless Bluetooth Module
// By Jonathan Matarazzi
// May 5 2022
#include<Wire.h>
#include<BluetoothSerial.h>
#include "Esp.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "driver\rtc_io.h"

#define PAIRING_TIME 10000
#define PAIR_BLINK_INTERVAL 250

#define BLUE_PIN 13
#define RED_PIN 12
#define MPU_POWER_PIN 19
#define PAIR_PIN 26
#define POWER_OFF_PIN 32
#define POWER_OFF_PIN_GPIO GPIO_NUM_32
#define WIRED_MODE_PIN 33
#define WIRELESS_MODE_PIN 25

BluetoothSerial SerialBT;

const int MPU_addr = 0x68; // I2C address of the MPU-6050
byte output[12];

bool transmitMode;

TaskHandle_t readIMU, clientHandler, pairingTask;


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

  //Enable bluetooth
  startBluetoothStack();
  SerialBT.begin("WirelessIMU-" + getBluetoothAddress());
  discoverable(false);

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

  xTaskCreatePinnedToCore(
    pairingCode,
    "ParingTask",
    10000,
    NULL,
    1,
    &pairingTask,
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
      if (SerialBT.available() > 0) {
        SerialBT.read();
        SerialBT.write(output, 12);
      }
    } else {
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

void pairingCode ( void * parameters) {
  for (;;) {
    if (transmitMode == true) {
      if (digitalRead(PAIR_PIN) == LOW) {
        discoverable(true);
        for (int i = 0; i < (PAIRING_TIME / PAIR_BLINK_INTERVAL); i++) {
          digitalWrite(BLUE_PIN, LOW);
          delay(PAIR_BLINK_INTERVAL / 2);
          digitalWrite(BLUE_PIN, HIGH);
          delay(PAIR_BLINK_INTERVAL / 2);
        }
        discoverable(false);
      } else {
        digitalWrite(BLUE_PIN, LOW);
      }
    } else {
      digitalWrite(BLUE_PIN, HIGH);
    }
  }
}

void startBluetoothStack() {
  btStart();
  esp_bluedroid_init();
  esp_bluedroid_enable();
}

String getBluetoothAddress() {
  const uint8_t* address = esp_bt_dev_get_address();
  String addrString = String();
  for (int i = 0; i < 2; i++) {
    char str[3];
    sprintf(str, "%02X", (int)address[i+4]);
    addrString = String(addrString + str);
  }
  return addrString;
}

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

void discoverable(bool dis) {
  if (dis) {
    esp_bt_gap_set_scan_mode(ESP_BT_SCAN_MODE_CONNECTABLE_DISCOVERABLE);
  } else {
    esp_bt_gap_set_scan_mode(ESP_BT_SCAN_MODE_CONNECTABLE);
  }
}


void loop() {
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
