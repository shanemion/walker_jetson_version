#include <Arduino.h>
#include <Wire.h>

#include "SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h"
#include "SparkFun_BNO08x_Arduino_Library.h"

static const uint8_t TCA_ADDR = 0x70;
static const uint8_t FORCE_PORTS[4] = {0, 1, 2, 3};
static const uint8_t IMU_PORT = 7;
static const uint32_t BAUD = 115200;
static const uint32_t SAMPLE_PERIOD_US = 20000;

NAU7802 scales[4];
BNO08x imu;

bool scaleValid[4] = {false, false, false, false};
bool imuValid = false;
long forceRaw[4] = {0, 0, 0, 0};
bool forceSampleValid[4] = {false, false, false, false};

float quatW = 1.0f;
float quatX = 0.0f;
float quatY = 0.0f;
float quatZ = 0.0f;
float accelX = 0.0f;
float accelY = 0.0f;
float accelZ = 0.0f;
float gyroX = 0.0f;
float gyroY = 0.0f;
float gyroZ = 0.0f;

uint32_t nextSampleUs = 0;
uint32_t seq = 0;
bool muxPresent = false;

bool selectMuxPort(uint8_t port) {
  if (port > 7) {
    return false;
  }
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << port);
  return Wire.endTransmission() == 0;
}

void disableMux() {
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(0);
  Wire.endTransmission();
}

bool i2cAddressResponds(uint8_t address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}

void scanMainBus() {
  Serial.println("main i2c scan begin");
  muxPresent = false;
  for (uint8_t address = 1; address < 127; ++address) {
    if (i2cAddressResponds(address)) {
      Serial.print("main i2c: 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.println(address, HEX);
      if (address == TCA_ADDR) {
        muxPresent = true;
      }
    }
  }
  Serial.print("mux_present ");
  Serial.println(muxPresent ? "true" : "false");
  Serial.println("main i2c scan end");
}

void scanMuxPorts() {
  Serial.println("forcewalker teensy force/imu bridge");
  Serial.println("mux scan begin");
  for (uint8_t port = 0; port < 8; ++port) {
    Serial.print("mux port ");
    Serial.print(port);
    Serial.print(":");
    if (!selectMuxPort(port)) {
      Serial.println(" tca_select_failed");
      continue;
    }
    bool any = false;
    for (uint8_t address = 1; address < 127; ++address) {
      if (i2cAddressResponds(address)) {
        Serial.print(" 0x");
        if (address < 16) {
          Serial.print("0");
        }
        Serial.print(address, HEX);
        any = true;
      }
    }
    if (!any) {
      Serial.print(" empty");
    }
    Serial.println();
  }
  disableMux();
  Serial.println("mux scan end");
}

void setupScales() {
  for (uint8_t index = 0; index < 4; ++index) {
    uint8_t port = FORCE_PORTS[index];
    if (!selectMuxPort(port)) {
      scaleValid[index] = false;
      continue;
    }
    scaleValid[index] = scales[index].begin(Wire);
    if (scaleValid[index]) {
      scales[index].setSampleRate(NAU7802_SPS_80);
      scales[index].calibrateAFE();
    }
    Serial.print("force channel ");
    Serial.print(index);
    Serial.print(" mux_port ");
    Serial.print(port);
    Serial.print(" valid ");
    Serial.println(scaleValid[index] ? "true" : "false");
  }
}

void setupImu() {
  if (!selectMuxPort(IMU_PORT)) {
    imuValid = false;
  } else {
    imuValid = imu.begin(BNO08x_DEFAULT_ADDRESS, Wire);
    if (imuValid) {
      imu.enableRotationVector(20);
      imu.enableAccelerometer(20);
      imu.enableGyro(20);
    }
  }
  Serial.print("imu mux_port ");
  Serial.print(IMU_PORT);
  Serial.print(" valid ");
  Serial.println(imuValid ? "true" : "false");
}

void setup() {
  Serial.begin(BAUD);
  Wire.begin();
  Wire.setClock(100000);
  delay(500);

  scanMainBus();
  scanMuxPorts();
  setupScales();
  setupImu();
  disableMux();
  nextSampleUs = micros();
}

void readForces() {
  for (uint8_t index = 0; index < 4; ++index) {
    forceSampleValid[index] = false;
    if (!scaleValid[index]) {
      continue;
    }
    if (!selectMuxPort(FORCE_PORTS[index])) {
      scaleValid[index] = false;
      continue;
    }
    if (scales[index].available()) {
      forceRaw[index] = scales[index].getReading();
      forceSampleValid[index] = true;
    }
  }
}

void readImu() {
  if (!imuValid) {
    return;
  }
  if (!selectMuxPort(IMU_PORT)) {
    imuValid = false;
    return;
  }
  if (imu.getSensorEvent()) {
    quatW = imu.getQuatReal();
    quatX = imu.getQuatI();
    quatY = imu.getQuatJ();
    quatZ = imu.getQuatK();
    accelX = imu.getAccelX();
    accelY = imu.getAccelY();
    accelZ = imu.getAccelZ();
    gyroX = imu.getGyroX();
    gyroY = imu.getGyroY();
    gyroZ = imu.getGyroZ();
  }
}

void printJsonSample(uint32_t tUs) {
  Serial.print("{\"seq\":");
  Serial.print(seq++);
  Serial.print(",\"t_us\":");
  Serial.print(tUs);
  Serial.print(",\"force_raw\":[");
  for (uint8_t index = 0; index < 4; ++index) {
    if (index > 0) {
      Serial.print(",");
    }
    if (scaleValid[index]) {
      Serial.print(forceRaw[index]);
    } else {
      Serial.print("null");
    }
  }
  Serial.print("],\"force_valid\":[");
  for (uint8_t index = 0; index < 4; ++index) {
    if (index > 0) {
      Serial.print(",");
    }
    Serial.print((scaleValid[index] && forceSampleValid[index]) ? "true" : "false");
  }
  Serial.print("],\"imu\":{\"valid\":");
  Serial.print(imuValid ? "true" : "false");
  Serial.print(",\"q\":[");
  Serial.print(quatW, 7);
  Serial.print(",");
  Serial.print(quatX, 7);
  Serial.print(",");
  Serial.print(quatY, 7);
  Serial.print(",");
  Serial.print(quatZ, 7);
  Serial.print("],\"accel_mps2\":[");
  Serial.print(accelX, 6);
  Serial.print(",");
  Serial.print(accelY, 6);
  Serial.print(",");
  Serial.print(accelZ, 6);
  Serial.print("],\"gyro_radps\":[");
  Serial.print(gyroX, 6);
  Serial.print(",");
  Serial.print(gyroY, 6);
  Serial.print(",");
  Serial.print(gyroZ, 6);
  Serial.print("]},\"status\":{\"mux_addr\":");
  Serial.print(TCA_ADDR);
  Serial.print(",\"mux_present\":");
  Serial.print(muxPresent ? "true" : "false");
  Serial.print(",\"force_present\":[");
  for (uint8_t index = 0; index < 4; ++index) {
    if (index > 0) {
      Serial.print(",");
    }
    Serial.print(scaleValid[index] ? "true" : "false");
  }
  Serial.print("]}}\n");
}

void loop() {
  uint32_t now = micros();
  if ((int32_t)(now - nextSampleUs) < 0) {
    return;
  }
  nextSampleUs += SAMPLE_PERIOD_US;

  readForces();
  readImu();
  printJsonSample(now);
}
