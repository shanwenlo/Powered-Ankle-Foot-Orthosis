// This code is testing one full ortoisis module
// Motor is set to actuate (with feedback) with heel/toe strike
// All sensors are tested and readings are printed out into .CSV

#include <SPI.h>
#include <Protocentral_ADS1220.h>
#include <Encoder.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <SD.h>
#include <VescUart.h>

// =====================================================
// SETTINGS
// =====================================================
const float SAMPLE_HZ = 100.0;  // set sampling rate of all sensors here
const uint32_t SAMPLE_PERIOD_US = 1000000UL / SAMPLE_HZ;
const uint32_t TEST_DURATION_MS = 10000;  //set test duration here

const char CSV_FILENAME[] = "DATA.csv";

const int LED_PIN = 0;

// =====================================================
// ENCODERS
// =====================================================
Encoder enc1(22, 23);   // A=22, B=23
Encoder enc2(20, 21);   // A=20, B=21
Encoder enc5(39, 40);   // A=39, B=40, motor encoder
Encoder enc3(14, 15);   // A=14, B=15
Encoder enc4(16, 17);   // A=16, B=17
Encoder enc6(2, 3);   // A=2, B=3, motor encoder

const float COUNTS_PER_REV = 2000.0;   // 500 PPR * 4
const float COUNTS_PER_DEG = COUNTS_PER_REV / 360.0;

long enc5_home = 0;

// =====================================================
// MOTOR / VESC
// =====================================================
VescUart vesc;

// Direction signs. If the motor moves the wrong way, flip these between 1 and -1.
const int LC1_TENSION_MOTOR_DIR = 1;
const int LC2_TENSION_MOTOR_DIR = -1;   // Opposite direction from LC1. Flip if LC2 pulls the wrong way.
const int HOME_MOTOR_DIR = 1;

// Tension control behavior
extern float lc1_N;   // load cell 1 force in N, defined in the ADS1220 section below
extern float lc2_N;   // load cell 2 force in N, defined in the ADS1220 section below
const float TARGET_TENSION_N = 50.0;         // set target tension for actuation
const float KP_CURRENT_PER_N = 0.05;          // Tune proportionality constant of controller
const float MAX_TENSION_CURRENT_A = 3.0;      // Current limit during tension control
const float TENSION_DEADBAND_N = 0.3;         // Close enough to target tension

// Return-home behavior
const float HOME_RETURN_CURRENT_A = 1.0;      // Fixed current for fast return home
const long HOME_TOLERANCE_COUNTS = 75;        // 75 counts = 13.5 deg for 2000 counts/rev

// Encoder travel safety limit relative to startup home
const float MAX_TRAVEL_DEG = 7200.0; //7200deg = 1 gearbox revolution
const long MAX_TRAVEL_COUNTS = (long)(MAX_TRAVEL_DEG * COUNTS_PER_DEG);

float motorCommand_A = 0.0;
int homeReturnSign = 0;

// =====================================================
// FSR
// =====================================================
const int FSR3_PIN = 24; //Left Heel
const int FSR4_PIN = 25; //Left Toe
const int FSR1_PIN = 26; //Left Heel
const int FSR2_PIN = 27; //Left Toe

// Press turns ON above 650; release turns OFF below 550.
const int FSR_PRESS_THRESHOLD = 650;
const int FSR_RELEASE_THRESHOLD = 550;

int fsr1_raw = 0;
int fsr2_raw = 0;
bool fsr1Pressed = false;
bool fsr2Pressed = false;

void updateFSRPressedStates() {
  if (!fsr1Pressed && fsr1_raw >= FSR_PRESS_THRESHOLD) fsr1Pressed = true;
  if ( fsr1Pressed && fsr1_raw <= FSR_RELEASE_THRESHOLD) fsr1Pressed = false;

  if (!fsr2Pressed && fsr2_raw >= FSR_PRESS_THRESHOLD) fsr2Pressed = true;
  if ( fsr2Pressed && fsr2_raw <= FSR_RELEASE_THRESHOLD) fsr2Pressed = false;
}

bool anyFSRPressed() {
  return fsr1Pressed || fsr2Pressed;
}

bool bothFSRsPressed() {
  return fsr1Pressed && fsr2Pressed;
}

bool noFSRsPressed() {
  return !fsr1Pressed && !fsr2Pressed;
}

// =====================================================
// MOTOR STATE
// =====================================================
enum MotorState {
  WAIT_FOR_FSR1 = 0,       // Waiting for the first side to activate
  TENSION_LC1 = 1,         // FSR1 pressed: tension LC1 to target
  TENSION_LC2 = 2,         // FSR2 pressed after FSR1: tension LC2 to target
  RETURN_HOME = 3,         // Return home once after sequence ends
  HOLD_HOME_ZERO_CURRENT = 4
};

MotorState motorState = WAIT_FOR_FSR1;

float clampFloat(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

void stopMotor() {
  motorCommand_A = 0.0;
  vesc.setCurrent(0.0);
}

long getEnc5RelativeCounts() {
  return enc5.read() - enc5_home;
}

float getEnc5RelativeDeg() {
  return getEnc5RelativeCounts() * (360.0 / COUNTS_PER_REV);
}

bool beyondTravelLimit() {
  long relativeCounts = getEnc5RelativeCounts();
  return abs(relativeCounts) >= MAX_TRAVEL_COUNTS;
}

void commandTensionCurrent(float measuredTension_N, int motorDir) {
  float error_N = TARGET_TENSION_N - measuredTension_N;

  if (error_N <= TENSION_DEADBAND_N) {
    motorCommand_A = 0.0;
  } else {
    motorCommand_A = KP_CURRENT_PER_N * error_N;
    motorCommand_A = clampFloat(motorCommand_A, 0.0, MAX_TENSION_CURRENT_A);
    motorCommand_A *= motorDir;
  }

  vesc.setCurrent(motorCommand_A);
}

void updateMotor() {
  long enc5_counts = enc5.read();
  long relativeCounts = enc5_counts - enc5_home;
  long errorCounts = enc5_home - enc5_counts;

  switch (motorState) {
    case WAIT_FOR_FSR1:
      // FSR2 alone does nothing. The cycle must start with FSR1.
      stopMotor();

      if (fsr1Pressed) {
        homeReturnSign = 0;
        motorState = TENSION_LC1;
      }
      break;

    case TENSION_LC1:
      // LC1 activates first and keeps controlling LC1 until FSR2 is pressed.
      // Once FSR2 is pressed, switch to LC2 and do NOT keep trying to hold LC1.
      if (fsr2Pressed) {
        motorState = TENSION_LC2;
        break;
      }

      // If FSR1 is released before FSR2 ever activates, end the cycle and go home.
      if (!fsr1Pressed) {
        homeReturnSign = 0;
        motorState = RETURN_HOME;
        break;
      }

      if (abs(relativeCounts) >= MAX_TRAVEL_COUNTS) {
        stopMotor();
        break;
      }

      commandTensionCurrent(lc1_N, LC1_TENSION_MOTOR_DIR);
      break;

    case TENSION_LC2:
      // LC2 stays active until FSR2 is released.
      // FSR1 is ignored in this state, so the controller cannot jump back to LC1.
      if (!fsr2Pressed) {
        homeReturnSign = 0;
        motorState = RETURN_HOME;
        break;
      }

      if (abs(relativeCounts) >= MAX_TRAVEL_COUNTS) {
        stopMotor();
        break;
      }

      commandTensionCurrent(lc2_N, LC2_TENSION_MOTOR_DIR);
      break;

    case RETURN_HOME:
      if (abs(errorCounts) <= HOME_TOLERANCE_COUNTS) {
        stopMotor();
        homeReturnSign = 0;
        motorState = HOLD_HOME_ZERO_CURRENT;
        break;
      }

      if (homeReturnSign == 0) {
        homeReturnSign = (errorCounts > 0) ? 1 : -1;
      }

      if ((homeReturnSign == 1 && errorCounts < 0) ||
          (homeReturnSign == -1 && errorCounts > 0)) {
        stopMotor();
        homeReturnSign = 0;
        motorState = HOLD_HOME_ZERO_CURRENT;
        break;
      }

      motorCommand_A = homeReturnSign * HOME_RETURN_CURRENT_A * HOME_MOTOR_DIR;
      vesc.setCurrent(motorCommand_A);
      break;

    case HOLD_HOME_ZERO_CURRENT:
      // Stay at 0 A after homing. A new sequence must start with FSR1 again.
      stopMotor();

      if (fsr1Pressed) {
        homeReturnSign = 0;
        motorState = TENSION_LC1;
      }
      break;
  }
}

void returnMotorHomeBlocking() {
  const uint32_t HOME_TIMEOUT_MS = 5000;
  uint32_t startTime = millis();

  int returnSign = 0;

  while (true) {
    long pos = enc5.read();
    long errorCounts = enc5_home - pos;

    if (abs(errorCounts) <= HOME_TOLERANCE_COUNTS) {
      break;
    }

    if (returnSign == 0) {
      returnSign = (errorCounts > 0) ? 1 : -1;
    }

    if ((returnSign == 1 && errorCounts < 0) ||
        (returnSign == -1 && errorCounts > 0)) {
      break;
    }

    motorCommand_A = returnSign * HOME_RETURN_CURRENT_A * HOME_MOTOR_DIR;
    vesc.setCurrent(motorCommand_A);

    if (millis() - startTime > HOME_TIMEOUT_MS) {
      break;
    }

    delay(5);
  }

  stopMotor();
}

// =====================================================
// BNO085
// =====================================================
#define BNO08X_INT 36

Adafruit_BNO08x bno08x(-1);
sh2_SensorValue_t sensorValue;

float ax = 0.0;
float ay = 0.0;
float az = 0.0;

void setBNOReports() {
  if (!bno08x.enableReport(SH2_LINEAR_ACCELERATION, 10000)) {
    Serial.println("Could not enable BNO085 linear acceleration");
    while (1) delay(10);
  }
}

void updateBNO() {
  if (bno08x.wasReset()) {
    setBNOReports();
  }

  if (digitalRead(BNO08X_INT) == LOW) {
    while (bno08x.getSensorEvent(&sensorValue)) {
      if (sensorValue.sensorId == SH2_LINEAR_ACCELERATION) {
        ax = sensorValue.un.linearAcceleration.x;
        ay = sensorValue.un.linearAcceleration.y;
        az = sensorValue.un.linearAcceleration.z;
      }
    }
  }
}

// =====================================================
// ADS1220 / LOAD CELLS
// =====================================================
Protocentral_ADS1220 ads;

const uint8_t ADS_CS2   = 41; //Right
const uint8_t ADS_DRDY2 = 38; //Right
const uint8_t ADS_CS1   = 4; //Left
const uint8_t ADS_DRDY1 = 5; //Left

int32_t tare1 = 0;
int32_t tare2 = 0;

const int32_t deadband1 = 400;
const int32_t deadband2 = 400;

const float lcAlpha = 0.8;

float filteredLC1_counts = 0.0;
float filteredLC2_counts = 0.0;

int32_t lc1_raw_adj = 0;
int32_t lc2_raw_adj = 0;

float lc1_N = 0.0;
float lc2_N = 0.0;

const uint32_t LC_UPDATE_PERIOD_MS = 20;
uint32_t lastLCUpdateMs = 0;

int32_t applyDeadband(int32_t value, int32_t threshold) {
  if (abs(value) < threshold) return 0;
  return value;
}

float countsToNewton(float counts) {
  return (((counts * -0.0140883446) - 0.87026) / 128.0);
}

void tareBothLoadCells() {
  Serial.println("Taring load cells. Keep both unloaded.");

  int64_t sum1 = 0;
  int64_t sum2 = 0;
  const int nSamples = 20;

  for (int i = 0; i < nSamples; i++) {
    ads.select_mux_channels(MUX_AIN0_AIN1);
    delayMicroseconds(200);
    sum1 += ads.Read_SingleShot_WaitForData();

    ads.select_mux_channels(MUX_AIN2_AIN3);
    delayMicroseconds(200);
    sum2 += ads.Read_SingleShot_WaitForData();
  }

  tare1 = sum1 / nSamples;
  tare2 = sum2 / nSamples;

  filteredLC1_counts = 0.0;
  filteredLC2_counts = 0.0;

  Serial.print("Tare 1: ");
  Serial.println(tare1);
  Serial.print("Tare 2: ");
  Serial.println(tare2);
}

void updateLoadCells() {
  ads.select_mux_channels(MUX_AIN0_AIN1);
  delayMicroseconds(200);
  int32_t raw1 = ads.Read_SingleShot_WaitForData();

  ads.select_mux_channels(MUX_AIN2_AIN3);
  delayMicroseconds(200);
  int32_t raw2 = ads.Read_SingleShot_WaitForData();

  lc1_raw_adj = raw1 - tare1;
  lc2_raw_adj = raw2 - tare2;

  int32_t db1 = applyDeadband(lc1_raw_adj, deadband1);
  int32_t db2 = applyDeadband(lc2_raw_adj, deadband2);

  filteredLC1_counts = lcAlpha * db1 + (1.0 - lcAlpha) * filteredLC1_counts;
  filteredLC2_counts = lcAlpha * db2 + (1.0 - lcAlpha) * filteredLC2_counts;

  lc1_N = countsToNewton(filteredLC1_counts);
  lc2_N = countsToNewton(filteredLC2_counts);
}

// =====================================================
// SD
// =====================================================
File logFile;

// =====================================================
// SETUP
// =====================================================
void setup() {
  Serial.begin(250000);
  Serial7.begin(115200);   // VESC UART on Serial2 //Left is on serial7
  delay(1500);

  Serial.println("Starting combined logger...");

  vesc.setSerialPort(&Serial7);
  stopMotor();

  // FSR
  analogReadResolution(10);
  pinMode(FSR1_PIN, INPUT);
  pinMode(FSR2_PIN, INPUT);

  // Encoders
  pinMode(22, INPUT_PULLUP);
  pinMode(23, INPUT_PULLUP);
  pinMode(20, INPUT_PULLUP);
  pinMode(21, INPUT_PULLUP);
  pinMode(39, INPUT_PULLUP);
  pinMode(40, INPUT_PULLUP);

  enc1.write(0);
  enc2.write(0);
  enc5.write(0);

  enc5_home = enc5.read();

  // BNO085
  Wire.begin();
  Wire.setClock(400000);
  pinMode(BNO08X_INT, INPUT);

  if (!bno08x.begin_I2C()) {
    Serial.println("Failed to find BNO085");
    while (1) delay(10);
  }

  setBNOReports();

  // ADS1220
  pinMode(ADS_CS1, OUTPUT);
  pinMode(ADS_DRDY1, INPUT);

  SPI.begin();

  ads.begin(ADS_CS1, ADS_DRDY1);
  ads.set_pga_gain(PGA_GAIN_128);
  ads.set_data_rate(DR_330SPS);
  ads.set_conv_mode_single_shot();

  tareBothLoadCells();

  // SD Card
  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("SD card failed.");
    while (1) delay(10);
  }

  SD.remove(CSV_FILENAME);
  logFile = SD.open(CSV_FILENAME, FILE_WRITE);

  if (!logFile) {
    Serial.println("Could not open CSV file.");
    while (1) delay(10);
  }

  logFile.println(
    "time_ms,"
    "ax,ay,az,"
    "fsr1_raw,fsr2_raw,"
    "enc1_counts,enc2_counts,enc5_counts,"
    "enc1_deg,enc2_deg,enc5_deg,enc5_relative_deg,"
    "lc1_raw_adj,lc2_raw_adj,lc1_N,lc2_N,"
    "motor_state,motor_current_A"
  );

  logFile.flush();

  Serial.println("Logging started.");
}

// =====================================================
// LOOP
// =====================================================
void loop() {
  static uint32_t startMs = millis();
  static uint32_t nextSampleUs = micros();
  static uint32_t sampleCounter = 0;
  static bool finished = false;

  if (finished) {
    stopMotor();
    delay(1000);
    return;
  }

  uint32_t nowMs = millis();

  if (nowMs - startMs >= TEST_DURATION_MS) {
    returnMotorHomeBlocking();

    logFile.flush();
    logFile.close();

    Serial.println("Done. Motor returned home. Saved DATA.csv");
    finished = true;
    digitalWrite(LED_PIN, LOW);    // LED OFF when finished
    return;
  }

  // Always read FSRs and load cell before updating motor state.
  fsr1_raw = analogRead(FSR1_PIN);
  fsr2_raw = analogRead(FSR2_PIN);
  updateFSRPressedStates();

  if (nowMs - lastLCUpdateMs >= LC_UPDATE_PERIOD_MS) {
    lastLCUpdateMs = nowMs;
    updateLoadCells();
  }

  updateMotor();

  uint32_t nowUs = micros();

  if ((int32_t)(nowUs - nextSampleUs) >= 0) {
    nextSampleUs += SAMPLE_PERIOD_US;

    updateBNO();

    long enc1_counts = enc1.read();
    long enc2_counts = enc2.read();
    long enc5_counts = enc5.read();

    float enc1_deg = enc1_counts * (360.0 / COUNTS_PER_REV);
    float enc2_deg = enc2_counts * (360.0 / COUNTS_PER_REV);
    float enc5_deg = enc5_counts * (360.0 / COUNTS_PER_REV);
    float enc5_relative_deg = (enc5_counts - enc5_home) * (360.0 / COUNTS_PER_REV);

    logFile.print(nowMs - startMs);
    logFile.print(",");

    logFile.print(ax, 4);
    logFile.print(",");
    logFile.print(ay, 4);
    logFile.print(",");
    logFile.print(az, 4);
    logFile.print(",");

    logFile.print(fsr1_raw);
    logFile.print(",");
    logFile.print(fsr2_raw);
    logFile.print(",");

    logFile.print(enc1_counts);
    logFile.print(",");
    logFile.print(enc2_counts);
    logFile.print(",");
    logFile.print(enc5_counts);
    logFile.print(",");

    logFile.print(enc1_deg, 3);
    logFile.print(",");
    logFile.print(enc2_deg, 3);
    logFile.print(",");
    logFile.print(enc5_deg, 3);
    logFile.print(",");
    logFile.print(enc5_relative_deg, 3);
    logFile.print(",");

    logFile.print(lc1_raw_adj);
    logFile.print(",");
    logFile.print(lc2_raw_adj);
    logFile.print(",");
    logFile.print(lc1_N, 4);
    logFile.print(",");
    logFile.print(lc2_N, 4);
    logFile.print(",");

    logFile.print((int)motorState);
    logFile.print(",");
    logFile.println(motorCommand_A, 4);

    sampleCounter++;

    if (sampleCounter % 100 == 0) {
      logFile.flush();
    }
  }
}
