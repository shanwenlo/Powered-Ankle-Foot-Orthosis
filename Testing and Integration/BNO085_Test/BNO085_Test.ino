// This code is to test the BNO085 IMU sensor connection over I2C

#include <Wire.h>
#include <Adafruit_BNO08x.h>

#define BNO08X_INT 36

Adafruit_BNO08x bno08x(-1);   // no reset pin used for I2C
sh2_SensorValue_t sensorValue;

void setReports() {
  Serial.println("Enabling accelerometer...");
  if (!bno08x.enableReport(SH2_ACCELEROMETER)) {
    Serial.println("Could not enable accelerometer");
    while (1) {
      delay(10);
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println("Starting BNO08x with INT pin");

  Wire.begin();
  Wire.setClock(400000);

  pinMode(BNO08X_INT, INPUT);

  if (!bno08x.begin_I2C()) {
    Serial.println("Failed to find BNO08x chip");
    while (1) {
      delay(10);
    }
  }

  Serial.println("BNO08x Found!");
  setReports();
}

void loop() {
  if (bno08x.wasReset()) {
    Serial.println("Sensor was reset");
    setReports();
  }

  // BNO08x INT is typically active LOW: LOW means new data is ready
  if (digitalRead(BNO08X_INT) == LOW) {
    if (bno08x.getSensorEvent(&sensorValue)) {
      if (sensorValue.sensorId == SH2_ACCELEROMETER) {
        Serial.print("Ax: ");
        Serial.print(sensorValue.un.accelerometer.x, 3);
        Serial.print("  Ay: ");
        Serial.print(sensorValue.un.accelerometer.y, 3);
        Serial.print("  Az: ");
        Serial.println(sensorValue.un.accelerometer.z, 3);
      }
    }
  }
}