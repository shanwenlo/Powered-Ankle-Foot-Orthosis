// This code is to test motor/VESC opperation with load cell readings as the feedback.


#include <SPI.h>
#include <Protocentral_ADS1220.h>
#include <VescUart.h>

Protocentral_ADS1220 ads;
VescUart vesc;

// Pins
const uint8_t CS_PIN = 6;
const uint8_t DRDY_PIN = 7;

// ---------- Load cell processing ----------
int32_t tare_offset = 0;
const int32_t deadband = 20;     // raw-count deadband
const float alpha = 0.7f;        // EMA filter, higher = faster
float filtered_value = 0.0f;

// Your calibration from counts -> Newtons
float countsToNewtons(float filteredCounts) {
  return -filteredCounts * 0.0147202224f - 0.5723824853f;
}

// ---------- Tension control ----------
const float finalTargetTension_N = 50.0f;   // desired final cable tension
float targetTension_N = 0.0f;              // ramped target

// Ramp settings
const float rampRate_N_per_sec = 1.0f;     // target rises at 1 N/sec

// PI gains: start conservative
float Kp = 0.1f;   // A per N
float Ki = 0.5f;   // A per (N*s)

// Current command limits
float maxCurrent = 2.0f;
float minCurrent = -2.0f;   // keep 0 if motor should only pull in one direction

float currentCommand = 0.0f;
float integralError = 0.0f;

// Optional tension deadband for control, in Newtons
const float tensionDeadband_N = 0.05f;

// Timing
unsigned long lastControlMicros = 0;
unsigned long lastPrintMillis = 0;

int32_t applyDeadband(int32_t value, int32_t threshold) {
  if (abs(value) < threshold) return 0;
  return value;
}

void tareLoadCell() {
  Serial.println("Taring... keep load cell unloaded");

  int64_t sum = 0;
  const int nSamples = 20;

  for (int i = 0; i < nSamples; i++) {
    sum += ads.Read_SingleShot_WaitForData();
  }

  tare_offset = sum / nSamples;
  filtered_value = 0.0f;

  Serial.print("Tare offset: ");
  Serial.println(tare_offset);
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);

  vesc.setSerialPort(&Serial1);
  ads.begin(CS_PIN, DRDY_PIN);

  delay(500);
  tareLoadCell();
  delay(500);

  lastControlMicros = micros();

  Serial.println("Closed-loop tension hold with soft ramp started");
  Serial.print("Final target tension (N): ");
  Serial.println(finalTargetTension_N, 2);
}

void loop() {
  // ---------- Compute dt ----------
  unsigned long nowMicros = micros();
  float dt = (nowMicros - lastControlMicros) * 1e-6f;
  lastControlMicros = nowMicros;

  if (dt <= 0.0f || dt > 0.1f) {
    dt = 0.01f;
  }

  // ---------- Soft ramp target ----------
  if (targetTension_N < finalTargetTension_N) {
    targetTension_N += rampRate_N_per_sec * dt;
    if (targetTension_N > finalTargetTension_N) {
      targetTension_N = finalTargetTension_N;
    }
  }

  // ---------- Read and filter load cell ----------
  int32_t raw = ads.Read_SingleShot_WaitForData();
  int32_t adjusted = raw - tare_offset;
  int32_t db = applyDeadband(adjusted, deadband);

  filtered_value = alpha * db + (1.0f - alpha) * filtered_value;

  float tension_N = countsToNewtons(filtered_value);

  if (tension_N < 0.0f) {
    tension_N = 0.0f;
  }

  // ---------- PI control ----------
  float error_N = targetTension_N - tension_N;

  if (fabs(error_N) < tensionDeadband_N) {
    error_N = 0.0f;
  }

  integralError += error_N * dt;

  // Anti-windup clamp
  const float integralLimit = 5.0f;
  if (integralError > integralLimit) integralError = integralLimit;
  if (integralError < -integralLimit) integralError = -integralLimit;

  currentCommand = -(Kp * error_N + Ki * integralError);

  if (currentCommand > maxCurrent) currentCommand = maxCurrent;
  if (currentCommand < minCurrent) currentCommand = minCurrent;

  // ---------- Send to VESC ----------
  vesc.setCurrent(currentCommand);

  // ---------- Debug print ----------
  if (millis() - lastPrintMillis >= 50) {
    lastPrintMillis = millis();

    Serial.print("Tension (N): ");
    Serial.print(tension_N, 2);
    Serial.print(" | Target (N): ");
    Serial.print(targetTension_N, 2);
    Serial.print(" | Current (A): ");
    Serial.println(currentCommand, 3);
  }
}