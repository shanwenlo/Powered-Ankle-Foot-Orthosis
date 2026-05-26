// This code is to test the load cell setup.
// The load cells are connected to an ADS1220 module which communicates with the Teensy via SPI.
// Load cells signal readings are converted to force in line 46.
// The load cells may need to be recalibrated and this conversion equation may need to be updated (see load cell calibration spreadsheet)

#include <SPI.h>
#include <Protocentral_ADS1220.h>

Protocentral_ADS1220 adsLeft;
Protocentral_ADS1220 adsRight;

// ---------- LED ----------
const uint8_t LED_PIN = 0;

// ---------- ADS pins ----------
// Left ADS module
const uint8_t ADS_LEFT_CS   = 4;
const uint8_t ADS_LEFT_DRDY = 5;

// Right ADS module
const uint8_t ADS_RIGHT_CS   = 41;
const uint8_t ADS_RIGHT_DRDY = 38;

// ---------- Tare values ----------
int32_t tare1 = 0;
int32_t tare2 = 0;
int32_t tare3 = 0;
int32_t tare4 = 0;

// ---------- Deadbands ----------
const int32_t deadband1 = 400;
const int32_t deadband2 = 400;
const int32_t deadband3 = 400;
const int32_t deadband4 = 400;

// ---------- Filtering ----------
const float alpha = 0.8f;

float filteredCounts1 = 0.0f;
float filteredCounts2 = 0.0f;
float filteredCounts3 = 0.0f;
float filteredCounts4 = 0.0f;

// ---------- Calibration ----------
float countsToNewtons(float counts) {
  return ((counts * -0.0140883446f) - 0.87026f) / 128.0f;
}

int32_t applyDeadband(int32_t value, int32_t threshold) {
  if (abs(value) < threshold) {
    return 0;
  }
  return value;
}

int32_t readChannel(Protocentral_ADS1220 &ads, uint8_t mux) {
  ads.select_mux_channels(mux);
  delayMicroseconds(200);
  return ads.Read_SingleShot_WaitForData();
}

void tareAllLoadCells() {
  Serial.println("Taring all 4 load cells... keep unloaded");

  int64_t sum1 = 0;
  int64_t sum2 = 0;
  int64_t sum3 = 0;
  int64_t sum4 = 0;

  const int nSamples = 20;

  for (int i = 0; i < nSamples; i++) {
    sum1 += readChannel(adsLeft, MUX_AIN0_AIN1);
    sum2 += readChannel(adsLeft, MUX_AIN2_AIN3);

    sum3 += readChannel(adsRight, MUX_AIN0_AIN1);
    sum4 += readChannel(adsRight, MUX_AIN2_AIN3);
  }

  tare1 = sum1 / nSamples;
  tare2 = sum2 / nSamples;
  tare3 = sum3 / nSamples;
  tare4 = sum4 / nSamples;

  filteredCounts1 = 0.0f;
  filteredCounts2 = 0.0f;
  filteredCounts3 = 0.0f;
  filteredCounts4 = 0.0f;

  Serial.println("Tare complete:");
  Serial.print("Tare 1: "); Serial.println(tare1);
  Serial.print("Tare 2: "); Serial.println(tare2);
  Serial.print("Tare 3: "); Serial.println(tare3);
  Serial.print("Tare 4: "); Serial.println(tare4);
}

void setup() {

  // ---------- LED ON ----------
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  Serial.begin(250000);
  delay(1500);

  pinMode(ADS_LEFT_CS, OUTPUT);
  pinMode(ADS_LEFT_DRDY, INPUT);

  pinMode(ADS_RIGHT_CS, OUTPUT);
  pinMode(ADS_RIGHT_DRDY, INPUT);

  digitalWrite(ADS_LEFT_CS, HIGH);
  digitalWrite(ADS_RIGHT_CS, HIGH);

  SPI.begin();

  adsLeft.begin(ADS_LEFT_CS, ADS_LEFT_DRDY);
  adsRight.begin(ADS_RIGHT_CS, ADS_RIGHT_DRDY);

  // Configure left ADS
  adsLeft.set_pga_gain(PGA_GAIN_128);
  adsLeft.set_data_rate(DR_175SPS);
  adsLeft.set_conv_mode_single_shot();

  // Configure right ADS
  adsRight.set_pga_gain(PGA_GAIN_128);
  adsRight.set_data_rate(DR_175SPS);
  adsRight.set_conv_mode_single_shot();

  tareAllLoadCells();

  Serial.println("Reading 4 load cells from 2 ADS1220 modules...");
}

void loop() {

  // Keep LED ON
  digitalWrite(LED_PIN, HIGH);

  // ---------- Read raw values ----------
  int32_t raw1 = readChannel(adsLeft, MUX_AIN0_AIN1);
  int32_t raw2 = readChannel(adsLeft, MUX_AIN2_AIN3);

  int32_t raw3 = readChannel(adsRight, MUX_AIN0_AIN1);
  int32_t raw4 = readChannel(adsRight, MUX_AIN2_AIN3);

  // ---------- Subtract tare ----------
  int32_t adjusted1 = raw1 - tare1;
  int32_t adjusted2 = raw2 - tare2;
  int32_t adjusted3 = raw3 - tare3;
  int32_t adjusted4 = raw4 - tare4;

  // ---------- Apply deadband ----------
  int32_t db1 = applyDeadband(adjusted1, deadband1);
  int32_t db2 = applyDeadband(adjusted2, deadband2);
  int32_t db3 = applyDeadband(adjusted3, deadband3);
  int32_t db4 = applyDeadband(adjusted4, deadband4);

  // ---------- Filter counts ----------
  filteredCounts1 = alpha * db1 + (1.0f - alpha) * filteredCounts1;
  filteredCounts2 = alpha * db2 + (1.0f - alpha) * filteredCounts2;
  filteredCounts3 = alpha * db3 + (1.0f - alpha) * filteredCounts3;
  filteredCounts4 = alpha * db4 + (1.0f - alpha) * filteredCounts4;

  // ---------- Convert to Newtons ----------
  float lc1_N = countsToNewtons(filteredCounts1);
  float lc2_N = countsToNewtons(filteredCounts2);
  float lc3_N = countsToNewtons(filteredCounts3);
  float lc4_N = countsToNewtons(filteredCounts4);

  // ---------- Print ----------
  Serial.print("LC1 raw: ");
  Serial.print(adjusted1);
  Serial.print(" | LC1 N: ");
  Serial.print(lc1_N, 2);

  Serial.print("    LC2 raw: ");
  Serial.print(adjusted2);
  Serial.print(" | LC2 N: ");
  Serial.print(lc2_N, 2);

  Serial.print("    LC3 raw: ");
  Serial.print(adjusted3);
  Serial.print(" | LC3 N: ");
  Serial.print(lc3_N, 2);

  Serial.print("    LC4 raw: ");
  Serial.print(adjusted4);
  Serial.print(" | LC4 N: ");
  Serial.println(lc4_N, 2);

  delay(5);
}