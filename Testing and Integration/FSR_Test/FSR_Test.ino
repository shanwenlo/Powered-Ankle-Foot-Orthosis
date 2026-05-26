// This code is to test FSR readings for heel/toe strike events


//const int FSR1_PIN = 24; //Right heel (limit ~700)
//const int FSR2_PIN = 25; //Right toe (limit=~500)

const int FSR1_PIN = 26; //Left heel
const int FSR2_PIN = 27; //Left toe

void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println("FSR reading started");
}

void loop() {
  int fsr1_raw = analogRead(FSR1_PIN);
  int fsr2_raw = analogRead(FSR2_PIN);

  // Convert to voltage (Teensy 4.1 uses 3.3V)
  float fsr1_voltage = fsr1_raw * (3.3 / 1023.0);
  float fsr2_voltage = fsr2_raw * (3.3 / 1023.0);

  Serial.print("FSR1: ");
  Serial.print(fsr1_raw);
  Serial.print(" (");
  Serial.print(fsr1_voltage, 2);
  Serial.print(" V)");

  Serial.print("   FSR2: ");
  Serial.print(fsr2_raw);
  Serial.print(" (");
  Serial.print(fsr2_voltage, 2);
  Serial.println(" V)");

  delay(5);
}