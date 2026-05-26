//  This code is used to collect IMU gait data without the orthosis (just IMU)


#include <Adafruit_BNO08x.h>
#include <SPI.h>
#include <SD.h>

#define BNO_CS     40
#define BNO_INT    2
#define BNO_RST    39

#define STATUS_LED 13
#define GREEN_LED  22

static const uint32_t LOG_DURATION_MS    = 50000;  // 40 seconds
static const uint32_t REPORT_INTERVAL_US = 10000;  // 100 Hz (10,000 us)

Adafruit_BNO08x bno(BNO_RST);
sh2_SensorValue_t val;

File logFile;

// Latest samples
float rawAx=0, rawAy=0, rawAz=0;
float linAx=0, linAy=0, linAz=0;
bool  haveRaw = false;
uint32_t rawT = 0;

bool makeFilename(char *outName, size_t outSize) {
  for (int i = 0; i < 100; i++) {
    snprintf(outName, outSize, "LOG%02d.CSV", i);
    if (!SD.exists(outName)) return true;
  }
  snprintf(outName, outSize, "LOG99.CSV");
  return true;
}

void blinkForever(uint16_t onMs, uint16_t offMs) {
  digitalWrite(GREEN_LED, LOW); // green only means “recording”
  while (1) {
    digitalWrite(STATUS_LED, HIGH); delay(onMs);
    digitalWrite(STATUS_LED, LOW);  delay(offMs);
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000) {}

  pinMode(STATUS_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  digitalWrite(STATUS_LED, LOW);
  digitalWrite(GREEN_LED, LOW);

  // --- SD init ---
  if (!SD.begin(BUILTIN_SDCARD)) {
    blinkForever(150, 150); // SD fail
  }

  // --- IMU init ---
  if (!bno.begin_SPI(BNO_CS, BNO_INT)) {
    blinkForever(600, 600); // IMU fail
  }

  // Enable BOTH reports
  if (!bno.enableReport(SH2_ACCELEROMETER, REPORT_INTERVAL_US)) {
    blinkForever(250, 750); // accel report fail
  }
  if (!bno.enableReport(SH2_LINEAR_ACCELERATION, REPORT_INTERVAL_US)) {
    blinkForever(750, 250); // linear report fail
  }

  // Create log file
  char fname[12];
  makeFilename(fname, sizeof(fname));

  logFile = SD.open(fname, FILE_WRITE);
  if (!logFile) {
    blinkForever(100, 900); // file open fail
  }

  // Header
  logFile.println("t_ms,rawAx,rawAy,rawAz,linAx,linAy,linAz,rawAge_ms");
  logFile.flush();

  // Optional “still” time to let fusion/bias settle
  Serial.println("Hold still (5s)...");
  delay(3000);

  // Blink 3 times right before starting
  for (int i = 0; i < 3; i++) {
    digitalWrite(STATUS_LED, HIGH); delay(300);
    digitalWrite(STATUS_LED, LOW);  delay(300);
  }

  // Solid ON during logging
  digitalWrite(STATUS_LED, HIGH);
  digitalWrite(GREEN_LED, HIGH);

  Serial.print("Logging to: ");
  Serial.println(fname);
  Serial.print("Walk now. Logging for ");
  Serial.print(LOG_DURATION_MS / 1000);
  Serial.println(" seconds...");
}

void loop() {
  static uint32_t tStart = millis();
  uint32_t tNow = millis();

  static uint32_t lastFlush = 0;
  static uint32_t rows = 0;

  if (tNow - tStart >= LOG_DURATION_MS) {
    digitalWrite(STATUS_LED, LOW);
    digitalWrite(GREEN_LED, LOW);
    logFile.flush();
    logFile.close();
    Serial.print("Done. Rows logged: ");
    Serial.println(rows);
    while (1) delay(1000);
  }

  if (bno.getSensorEvent(&val)) {
    if (val.sensorId == SH2_ACCELEROMETER) {
      rawAx = val.un.accelerometer.x;
      rawAy = val.un.accelerometer.y;
      rawAz = val.un.accelerometer.z;
      rawT = tNow;
      haveRaw = true;
    }

    if (val.sensorId == SH2_LINEAR_ACCELERATION) {
      linAx = val.un.linearAcceleration.x;
      linAy = val.un.linearAcceleration.y;
      linAz = val.un.linearAcceleration.z;

      if (haveRaw) {
        uint32_t rawAge = (tNow >= rawT) ? (tNow - rawT) : 0;

        logFile.print(tNow);
        logFile.print(",");
        logFile.print(rawAx, 6); logFile.print(",");
        logFile.print(rawAy, 6); logFile.print(",");
        logFile.print(rawAz, 6); logFile.print(",");
        logFile.print(linAx, 6); logFile.print(",");
        logFile.print(linAy, 6); logFile.print(",");
        logFile.print(linAz, 6); logFile.print(",");
        logFile.println(rawAge);

        rows++;

        // Flush less often (reduces timing hiccups)
        if (tNow - lastFlush > 2000) { // every 2 seconds
          logFile.flush();
          lastFlush = tNow;
        }
      }
    }
  }
}
