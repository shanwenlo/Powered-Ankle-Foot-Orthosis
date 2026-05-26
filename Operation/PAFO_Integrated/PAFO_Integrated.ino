// This is the full integrated code used for the gait trials

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
const float SAMPLE_HZ = 100.0; // set sampling rate for all sensors here
const uint32_t SAMPLE_PERIOD_US = 1000000UL / SAMPLE_HZ;
const uint32_t TEST_DURATION_MS = 30000;  // 30 sec sampling time (used to complete ~24 strides)

const uint32_t HEEL_STRIKE_DELAY_MS = 0;  // set delay for when inversion acutation occurs
const uint32_t TOE_STRIKE_DELAY_MS  = 50; // set delay for when eversion acutation occurs

const int LED_PIN = 0;

// =====================================================
// ENCODERS
// =====================================================
// RIGHT DEVICE
Encoder enc1(22, 23); // Right Subtalar Joint Axis
Encoder enc2(20, 21); // Right Talocrural Joint Axis
Encoder enc5(39, 40); // Right Motor Encoder

// LEFT DEVICE
Encoder enc3(14, 15); // Left Talocrural Joint Axis
Encoder enc4(16, 17);  // Left Subtalar Joint Axis
Encoder enc6(2, 3);  // Left Motor Encoder

const float COUNTS_PER_REV = 2000.0;
const float COUNTS_PER_DEG = COUNTS_PER_REV / 360.0;

// =====================================================
// FSR PINS
// =====================================================
const int FSR1_PIN = 24;  //right heel
const int FSR2_PIN = 25;  //right toe

const int FSR3_PIN = 26;  //left heel
const int FSR4_PIN = 27;  //left toe

const int FSR_PRESS_THRESHOLD = 650;
const int FSR_RELEASE_THRESHOLD = 550;

// =====================================================
// MOTOR / VESC
// =====================================================
VescUart vescRight;
VescUart vescLeft;

const float TARGET_TENSION_N = 25.0;  //set target tension here
const float KP_CURRENT_PER_N = 0.05;  // tune Kp here
const float MAX_TENSION_CURRENT_A = 2.5;  // set max current (should be <3A)
const float TENSION_DEADBAND_N = 0.3;

const float HOME_RETURN_CURRENT_A = 1.0;
const long HOME_TOLERANCE_COUNTS = 75;

const float MAX_TRAVEL_DEG = 7200.0;
const long MAX_TRAVEL_COUNTS = (long)(MAX_TRAVEL_DEG * COUNTS_PER_DEG);

// Flip these if direction is wrong
const int RIGHT_LC1_TENSION_DIR = 1;
const int RIGHT_LC2_TENSION_DIR = -1;
const int RIGHT_HOME_DIR = 1;

const int LEFT_LC1_TENSION_DIR = 1;
const int LEFT_LC2_TENSION_DIR = -1;
const int LEFT_HOME_DIR = 1;

enum MotorState {
  WAIT_FOR_FSR1_RELEASE = 0,
  WAIT_FOR_FSR1 = 1,
  HEEL_DELAY = 2,
  TENSION_LC1 = 3,
  TOE_DELAY = 4,
  TENSION_LC2 = 5,
  RETURN_HOME = 6,
  HOLD_HOME_ZERO_CURRENT = 7
};

struct DeviceControl {
  int fsr1Pin;
  int fsr2Pin;

  int fsr1Raw;
  int fsr2Raw;

  bool fsr1Pressed;
  bool fsr2Pressed;

  long motorHome;

  MotorState state;

  float motorCommandA;
  int homeReturnSign;

  int lc1MotorDir;
  int lc2MotorDir;
  int homeMotorDir;

  uint32_t heelStrikeTimeMs;
  uint32_t toeStrikeTimeMs;
};

DeviceControl rightDev = {
  FSR1_PIN, FSR2_PIN,
  0, 0,
  false, false,
  0,
  WAIT_FOR_FSR1_RELEASE,
  0.0, 0,
  RIGHT_LC1_TENSION_DIR,
  RIGHT_LC2_TENSION_DIR,
  RIGHT_HOME_DIR,
  0, 0
};

DeviceControl leftDev = {
  FSR3_PIN, FSR4_PIN,
  0, 0,
  false, false,
  0,
  WAIT_FOR_FSR1_RELEASE,
  0.0, 0,
  LEFT_LC1_TENSION_DIR,
  LEFT_LC2_TENSION_DIR,
  LEFT_HOME_DIR,
  0, 0
};

// =====================================================
// ADS1220 / LOAD CELLS
// =====================================================
Protocentral_ADS1220 adsRight;
Protocentral_ADS1220 adsLeft;

const uint8_t ADS_RIGHT_CS   = 41;
const uint8_t ADS_RIGHT_DRDY = 38;

const uint8_t ADS_LEFT_CS   = 4;
const uint8_t ADS_LEFT_DRDY = 5;

struct LoadCellPair {
  int32_t tare1;
  int32_t tare2;

  int32_t rawAdj1;
  int32_t rawAdj2;

  float filtered1Counts;
  float filtered2Counts;

  float lc1_N;
  float lc2_N;
};

LoadCellPair rightLC = {0, 0, 0, 0, 0.0, 0.0, 0.0, 0.0};
LoadCellPair leftLC  = {0, 0, 0, 0, 0.0, 0.0, 0.0, 0.0};

const int32_t deadband1 = 400;
const int32_t deadband2 = 400;
const float lcAlpha = 0.8;

// =====================================================
// BNO085
// =====================================================
#define BNO08X_INT 36

Adafruit_BNO08x bno08x(-1);
sh2_SensorValue_t sensorValue;

float ax = 0.0;
float ay = 0.0;
float az = 0.0;

// =====================================================
// SD
// =====================================================
File logFile;
char csvFilename[20];

// =====================================================
// HELPER FUNCTIONS
// =====================================================
float clampFloat(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

void createNewCsvFilename() {
  int fileNumber = 1;

  while (true) {
    snprintf(csvFilename, sizeof(csvFilename), "data%d.csv", fileNumber);

    if (!SD.exists(csvFilename)) {
      break;
    }

    fileNumber++;
  }
}

int32_t applyDeadband(int32_t value, int32_t threshold) {
  if (abs(value) < threshold) return 0;
  return value;
}

float countsToNewton(float counts) {
  return (((counts * -0.0140883446) - 0.87026) / 128.0);
}

// =====================================================
// MOTOR FUNCTIONS
// =====================================================
void stopDevice(DeviceControl &dev, VescUart &vesc) {
  dev.motorCommandA = 0.0;
  vesc.setCurrent(0.0);
}

void updateFSRPressedStates(DeviceControl &dev) {
  dev.fsr1Raw = analogRead(dev.fsr1Pin);
  dev.fsr2Raw = analogRead(dev.fsr2Pin);

  if (!dev.fsr1Pressed && dev.fsr1Raw >= FSR_PRESS_THRESHOLD) dev.fsr1Pressed = true;
  if ( dev.fsr1Pressed && dev.fsr1Raw <= FSR_RELEASE_THRESHOLD) dev.fsr1Pressed = false;

  if (!dev.fsr2Pressed && dev.fsr2Raw >= FSR_PRESS_THRESHOLD) dev.fsr2Pressed = true;
  if ( dev.fsr2Pressed && dev.fsr2Raw <= FSR_RELEASE_THRESHOLD) dev.fsr2Pressed = false;
}

void commandTensionCurrent(DeviceControl &dev, VescUart &vesc, float measuredTensionN, int motorDir) {
  float errorN = TARGET_TENSION_N - measuredTensionN;

  if (errorN <= TENSION_DEADBAND_N) {
    dev.motorCommandA = 0.0;
  } else {
    dev.motorCommandA = KP_CURRENT_PER_N * errorN;
    dev.motorCommandA = clampFloat(dev.motorCommandA, 0.0, MAX_TENSION_CURRENT_A);
    dev.motorCommandA *= motorDir;
  }

  vesc.setCurrent(dev.motorCommandA);
}

void updateDeviceMotor(
  DeviceControl &dev,
  Encoder &motorEnc,
  VescUart &vesc,
  float lc1_N,
  float lc2_N
) {
  long motorCounts = motorEnc.read();
  long relativeCounts = motorCounts - dev.motorHome;
  long errorCounts = dev.motorHome - motorCounts;

  switch (dev.state) {

    case WAIT_FOR_FSR1_RELEASE:
      stopDevice(dev, vesc);

      if (!dev.fsr1Pressed) {
        dev.state = WAIT_FOR_FSR1;
      }
      break;

    case WAIT_FOR_FSR1:
      stopDevice(dev, vesc);

      if (dev.fsr1Pressed) {
        dev.heelStrikeTimeMs = millis();
        dev.homeReturnSign = 0;
        dev.state = HEEL_DELAY;
      }
      break;

    case HEEL_DELAY:
      stopDevice(dev, vesc);

      if (!dev.fsr1Pressed) {
        dev.state = WAIT_FOR_FSR1;
        break;
      }

      if (millis() - dev.heelStrikeTimeMs >= HEEL_STRIKE_DELAY_MS) {
        dev.state = TENSION_LC1;
      }
      break;

    case TENSION_LC1:
      if (dev.fsr2Pressed) {
        dev.toeStrikeTimeMs = millis();
        dev.state = TOE_DELAY;
        break;
      }

      if (!dev.fsr1Pressed) {
        dev.homeReturnSign = 0;
        dev.state = RETURN_HOME;
        break;
      }

      if (abs(relativeCounts) >= MAX_TRAVEL_COUNTS) {
        stopDevice(dev, vesc);
        break;
      }

      commandTensionCurrent(dev, vesc, lc1_N, dev.lc1MotorDir);
      break;

    case TOE_DELAY:
      stopDevice(dev, vesc);

      if (!dev.fsr2Pressed) {
        dev.homeReturnSign = 0;
        dev.state = RETURN_HOME;
        break;
      }

      if (millis() - dev.toeStrikeTimeMs >= TOE_STRIKE_DELAY_MS) {
        dev.state = TENSION_LC2;
      }
      break;

    case TENSION_LC2:
      if (!dev.fsr2Pressed) {
        dev.homeReturnSign = 0;
        dev.state = RETURN_HOME;
        break;
      }

      if (abs(relativeCounts) >= MAX_TRAVEL_COUNTS) {
        stopDevice(dev, vesc);
        break;
      }

      commandTensionCurrent(dev, vesc, lc2_N, dev.lc2MotorDir);
      break;

    case RETURN_HOME:
      if (abs(errorCounts) <= HOME_TOLERANCE_COUNTS) {
        stopDevice(dev, vesc);
        dev.homeReturnSign = 0;
        dev.state = HOLD_HOME_ZERO_CURRENT;
        break;
      }

      if (dev.homeReturnSign == 0) {
        dev.homeReturnSign = (errorCounts > 0) ? 1 : -1;
      }

      if ((dev.homeReturnSign == 1 && errorCounts < 0) ||
          (dev.homeReturnSign == -1 && errorCounts > 0)) {
        stopDevice(dev, vesc);
        dev.homeReturnSign = 0;
        dev.state = HOLD_HOME_ZERO_CURRENT;
        break;
      }

      dev.motorCommandA = dev.homeReturnSign * HOME_RETURN_CURRENT_A * dev.homeMotorDir;
      vesc.setCurrent(dev.motorCommandA);
      break;

    case HOLD_HOME_ZERO_CURRENT:
      stopDevice(dev, vesc);

      if (!dev.fsr1Pressed) {
        dev.state = WAIT_FOR_FSR1;
      }
      break;
  }
}

// =====================================================
// BNO FUNCTIONS
// =====================================================
void setBNOReports() {
  if (!bno08x.enableReport(SH2_LINEAR_ACCELERATION, 10000)) {
    Serial.println("Could not enable BNO085 linear acceleration");
    while (1) delay(10);
  }
}

void updateBNO() {

  if (bno08x.wasReset()) {
    Serial.println("BNO085 RESET DETECTED");
    setBNOReports();
  }

  while (bno08x.getSensorEvent(&sensorValue)) {

    if (sensorValue.sensorId == SH2_LINEAR_ACCELERATION) {
      ax = sensorValue.un.linearAcceleration.x;
      ay = sensorValue.un.linearAcceleration.y;
      az = sensorValue.un.linearAcceleration.z;
    }
  }
}

// =====================================================
// LOAD CELL FUNCTIONS
// =====================================================
void tareLoadCells(Protocentral_ADS1220 &ads, LoadCellPair &lc, const char *name) {
  Serial.print("Taring ");
  Serial.print(name);
  Serial.println(" load cells. Keep unloaded.");

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

  lc.tare1 = sum1 / nSamples;
  lc.tare2 = sum2 / nSamples;

  lc.filtered1Counts = 0.0;
  lc.filtered2Counts = 0.0;

  Serial.print(name);
  Serial.print(" tare 1: ");
  Serial.println(lc.tare1);

  Serial.print(name);
  Serial.print(" tare 2: ");
  Serial.println(lc.tare2);
}

void updateLoadCells(Protocentral_ADS1220 &ads, LoadCellPair &lc) {
  ads.select_mux_channels(MUX_AIN0_AIN1);
  delayMicroseconds(200);
  int32_t raw1 = ads.Read_SingleShot_WaitForData();

  ads.select_mux_channels(MUX_AIN2_AIN3);
  delayMicroseconds(200);
  int32_t raw2 = ads.Read_SingleShot_WaitForData();

  lc.rawAdj1 = raw1 - lc.tare1;
  lc.rawAdj2 = raw2 - lc.tare2;

  int32_t db1 = applyDeadband(lc.rawAdj1, deadband1);
  int32_t db2 = applyDeadband(lc.rawAdj2, deadband2);

  lc.filtered1Counts = lcAlpha * db1 + (1.0 - lcAlpha) * lc.filtered1Counts;
  lc.filtered2Counts = lcAlpha * db2 + (1.0 - lcAlpha) * lc.filtered2Counts;

  lc.lc1_N = countsToNewton(lc.filtered1Counts);
  lc.lc2_N = countsToNewton(lc.filtered2Counts);
}

// =====================================================
// RETURN BOTH MOTORS HOME
// =====================================================
void returnBothMotorsHomeBlocking() {
  const uint32_t HOME_TIMEOUT_MS = 5000;
  uint32_t startTime = millis();

  rightDev.homeReturnSign = 0;
  leftDev.homeReturnSign = 0;

  bool rightDone = false;
  bool leftDone = false;

  while (!(rightDone && leftDone)) {
    long rightPos = enc5.read();
    long leftPos = enc6.read();

    long rightError = rightDev.motorHome - rightPos;
    long leftError = leftDev.motorHome - leftPos;

    if (!rightDone) {
      if (abs(rightError) <= HOME_TOLERANCE_COUNTS) {
        stopDevice(rightDev, vescRight);
        rightDone = true;
      } else {
        if (rightDev.homeReturnSign == 0) {
          rightDev.homeReturnSign = (rightError > 0) ? 1 : -1;
        }

        if ((rightDev.homeReturnSign == 1 && rightError < 0) ||
            (rightDev.homeReturnSign == -1 && rightError > 0)) {
          stopDevice(rightDev, vescRight);
          rightDone = true;
        } else {
          rightDev.motorCommandA = rightDev.homeReturnSign * HOME_RETURN_CURRENT_A * rightDev.homeMotorDir;
          vescRight.setCurrent(rightDev.motorCommandA);
        }
      }
    }

    if (!leftDone) {
      if (abs(leftError) <= HOME_TOLERANCE_COUNTS) {
        stopDevice(leftDev, vescLeft);
        leftDone = true;
      } else {
        if (leftDev.homeReturnSign == 0) {
          leftDev.homeReturnSign = (leftError > 0) ? 1 : -1;
        }

        if ((leftDev.homeReturnSign == 1 && leftError < 0) ||
            (leftDev.homeReturnSign == -1 && leftError > 0)) {
          stopDevice(leftDev, vescLeft);
          leftDone = true;
        } else {
          leftDev.motorCommandA = leftDev.homeReturnSign * HOME_RETURN_CURRENT_A * leftDev.homeMotorDir;
          vescLeft.setCurrent(leftDev.motorCommandA);
        }
      }
    }

    if (millis() - startTime > HOME_TIMEOUT_MS) {
      break;
    }

    delay(5);
  }

  stopDevice(rightDev, vescRight);
  stopDevice(leftDev, vescLeft);
}

// =====================================================
// SETUP
// =====================================================
void setup() {
  Serial.begin(250000);

  Serial2.begin(115200);
  Serial7.begin(115200);

  delay(1500);

  Serial.println("Starting two-device logger...");

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  vescRight.setSerialPort(&Serial2);
  vescLeft.setSerialPort(&Serial7);

  stopDevice(rightDev, vescRight);
  stopDevice(leftDev, vescLeft);

  analogReadResolution(10);

  pinMode(FSR1_PIN, INPUT);
  pinMode(FSR2_PIN, INPUT);
  pinMode(FSR3_PIN, INPUT);
  pinMode(FSR4_PIN, INPUT);

  pinMode(22, INPUT_PULLUP);
  pinMode(23, INPUT_PULLUP);
  pinMode(20, INPUT_PULLUP);
  pinMode(21, INPUT_PULLUP);
  pinMode(39, INPUT_PULLUP);
  pinMode(40, INPUT_PULLUP);

  pinMode(14, INPUT_PULLUP);
  pinMode(15, INPUT_PULLUP);
  pinMode(16, INPUT_PULLUP);
  pinMode(17, INPUT_PULLUP);
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);

  enc1.write(0);
  enc2.write(0);
  enc5.write(0);

  enc3.write(0);
  enc4.write(0);
  enc6.write(0);

  rightDev.motorHome = enc5.read();
  leftDev.motorHome = enc6.read();

  Wire.begin();
  Wire.setClock(100000);
  pinMode(BNO08X_INT, INPUT);

  if (!bno08x.begin_I2C()) {
    Serial.println("Failed to find BNO085");
    while (1) delay(10);
  }

  setBNOReports();

  pinMode(ADS_RIGHT_CS, OUTPUT);
  pinMode(ADS_RIGHT_DRDY, INPUT);
  pinMode(ADS_LEFT_CS, OUTPUT);
  pinMode(ADS_LEFT_DRDY, INPUT);

  SPI.begin();

  adsRight.begin(ADS_RIGHT_CS, ADS_RIGHT_DRDY);
  adsRight.set_pga_gain(PGA_GAIN_128);
  adsRight.set_data_rate(DR_330SPS);
  adsRight.set_conv_mode_single_shot();

  adsLeft.begin(ADS_LEFT_CS, ADS_LEFT_DRDY);
  adsLeft.set_pga_gain(PGA_GAIN_128);
  adsLeft.set_data_rate(DR_330SPS);
  adsLeft.set_conv_mode_single_shot();

  tareLoadCells(adsRight, rightLC, "RIGHT");
  tareLoadCells(adsLeft, leftLC, "LEFT");

  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("SD card failed.");
    while (1) delay(10);
  }

  createNewCsvFilename();
  logFile = SD.open(csvFilename, FILE_WRITE);

  if (!logFile) {
    Serial.println("Could not open CSV file.");
    while (1) delay(10);
  }

  logFile.println(
    "time_ms,"
    "ax,ay,az,"
    "right_fsr1_raw,right_fsr2_raw,left_fsr1_raw,left_fsr2_raw,"
    "right_enc1_counts,right_enc2_counts,right_motor_counts,"
    "left_enc1_counts,left_enc2_counts,left_motor_counts,"
    "right_enc1_deg,right_enc2_deg,right_motor_deg,right_motor_relative_deg,"
    "left_enc1_deg,left_enc2_deg,left_motor_deg,left_motor_relative_deg,"
    "right_lc1_raw_adj,right_lc2_raw_adj,right_lc1_N,right_lc2_N,"
    "left_lc1_raw_adj,left_lc2_raw_adj,left_lc1_N,left_lc2_N,"
    "right_motor_state,right_motor_current_A,"
    "left_motor_state,left_motor_current_A"
  );

  logFile.flush();

  Serial.print("Logging started. Saving to ");
  Serial.println(csvFilename);
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
    stopDevice(rightDev, vescRight);
    stopDevice(leftDev, vescLeft);
    delay(1000);
    return;
  }

  uint32_t nowMs = millis();

  // Keep BNO085 serviced continuously
  updateBNO();

  if (nowMs - startMs >= TEST_DURATION_MS) {
    returnBothMotorsHomeBlocking();

    logFile.flush();
    logFile.close();

    Serial.print("Done. Both motors returned home. Saved ");
    Serial.println(csvFilename);

    finished = true;
    digitalWrite(LED_PIN, LOW);
    return;
  }

  uint32_t nowUs = micros();

  if ((int32_t)(nowUs - nextSampleUs) >= 0) {
    nextSampleUs += SAMPLE_PERIOD_US;

    updateFSRPressedStates(rightDev);
    updateFSRPressedStates(leftDev);

    updateLoadCells(adsRight, rightLC);
    updateLoadCells(adsLeft, leftLC);

    updateDeviceMotor(rightDev, enc5, vescRight, rightLC.lc1_N, rightLC.lc2_N);
    updateDeviceMotor(leftDev, enc6, vescLeft, leftLC.lc1_N, leftLC.lc2_N);

    long rightEnc1Counts = enc1.read();
    long rightEnc2Counts = enc2.read();
    long rightMotorCounts = enc5.read();

    long leftEnc1Counts = enc3.read();
    long leftEnc2Counts = enc4.read();
    long leftMotorCounts = enc6.read();

    float rightEnc1Deg = rightEnc1Counts * (360.0 / COUNTS_PER_REV);
    float rightEnc2Deg = rightEnc2Counts * (360.0 / COUNTS_PER_REV);
    float rightMotorDeg = rightMotorCounts * (360.0 / COUNTS_PER_REV);
    float rightMotorRelativeDeg = (rightMotorCounts - rightDev.motorHome) * (360.0 / COUNTS_PER_REV);

    float leftEnc1Deg = leftEnc1Counts * (360.0 / COUNTS_PER_REV);
    float leftEnc2Deg = leftEnc2Counts * (360.0 / COUNTS_PER_REV);
    float leftMotorDeg = leftMotorCounts * (360.0 / COUNTS_PER_REV);
    float leftMotorRelativeDeg = (leftMotorCounts - leftDev.motorHome) * (360.0 / COUNTS_PER_REV);

    logFile.print(nowMs - startMs);
    logFile.print(",");

    logFile.print(ax, 4); logFile.print(",");
    logFile.print(ay, 4); logFile.print(",");
    logFile.print(az, 4); logFile.print(",");

    logFile.print(rightDev.fsr1Raw); logFile.print(",");
    logFile.print(rightDev.fsr2Raw); logFile.print(",");
    logFile.print(leftDev.fsr1Raw); logFile.print(",");
    logFile.print(leftDev.fsr2Raw); logFile.print(",");

    logFile.print(rightEnc1Counts); logFile.print(",");
    logFile.print(rightEnc2Counts); logFile.print(",");
    logFile.print(rightMotorCounts); logFile.print(",");

    logFile.print(leftEnc1Counts); logFile.print(",");
    logFile.print(leftEnc2Counts); logFile.print(",");
    logFile.print(leftMotorCounts); logFile.print(",");

    logFile.print(rightEnc1Deg, 3); logFile.print(",");
    logFile.print(rightEnc2Deg, 3); logFile.print(",");
    logFile.print(rightMotorDeg, 3); logFile.print(",");
    logFile.print(rightMotorRelativeDeg, 3); logFile.print(",");

    logFile.print(leftEnc1Deg, 3); logFile.print(",");
    logFile.print(leftEnc2Deg, 3); logFile.print(",");
    logFile.print(leftMotorDeg, 3); logFile.print(",");
    logFile.print(leftMotorRelativeDeg, 3); logFile.print(",");

    logFile.print(rightLC.rawAdj1); logFile.print(",");
    logFile.print(rightLC.rawAdj2); logFile.print(",");
    logFile.print(rightLC.lc1_N, 4); logFile.print(",");
    logFile.print(rightLC.lc2_N, 4); logFile.print(",");

    logFile.print(leftLC.rawAdj1); logFile.print(",");
    logFile.print(leftLC.rawAdj2); logFile.print(",");
    logFile.print(leftLC.lc1_N, 4); logFile.print(",");
    logFile.print(leftLC.lc2_N, 4); logFile.print(",");

    logFile.print((int)rightDev.state); logFile.print(",");
    logFile.print(rightDev.motorCommandA, 4); logFile.print(",");

    logFile.print((int)leftDev.state); logFile.print(",");
    logFile.println(leftDev.motorCommandA, 4);

    sampleCounter++;

    if (sampleCounter % 100 == 0) {
      logFile.flush();
    }
  }
}