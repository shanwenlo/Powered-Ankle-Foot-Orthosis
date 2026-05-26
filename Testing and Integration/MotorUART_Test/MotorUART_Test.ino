// This code is simply to test mootor operation using the VESC over UART connection
// spins motor for 1.5 seconds @ 1 A (make sure motor is unrestricted)

#include <VescUart.h>

VescUart vesc;

const float MOTOR_CURRENT_A = 1.0;
const unsigned long RUN_TIME_MS = 1500;

unsigned long startTime;
bool motorStopped = false;

void setup() {
  Serial.begin(115200);

  // Teensy 4.1 Serial2:
  // RX2 = pin 7
  // TX2 = pin 8
  Serial2.begin(115200);
  vesc.setSerialPort(&Serial2);

  delay(1000);

  startTime = millis();

  Serial.println("Motor spinning at 1 A for 1.5 seconds...");
}

void loop() {
  unsigned long elapsedTime = millis() - startTime;

  if (elapsedTime < RUN_TIME_MS) {
    vesc.setCurrent(MOTOR_CURRENT_A);
  } 
  else if (!motorStopped) {
    vesc.setCurrent(0.0);
    motorStopped = true;
    Serial.println("Motor stopped.");
  }
}