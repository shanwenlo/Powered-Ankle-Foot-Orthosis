// This code is to test encoder connection/readings


#include <Encoder.h>

// Encoder definitions (select desired encoder)
Encoder enc1(22, 23); //Right
Encoder enc2(20, 21); //Right
//Encoder enc1(14, 15); //Left
//Encoder enc2(16, 17); //Left
//Encoder enc5(39, 40); //Left motor encoder
//Encoder enc6(2, 3); //Right motor encoder


// Adjust this if needed after testing
int ppr = 500;
const float COUNTS_PER_REV = ppr*4;

void setup() {
  pinMode(22, INPUT_PULLUP);
  pinMode(23, INPUT_PULLUP);
  pinMode(20, INPUT_PULLUP);
  pinMode(21, INPUT_PULLUP);

  enc1.write(0);
  enc2.write(0);

  Serial.begin(115200);
  delay(2000);

  Serial.println("Encoder degree output");
}

void loop() {
  long counts1 = enc1.read();
  long counts2 = enc2.read();

  float deg1 = counts1 * (360.0 / COUNTS_PER_REV);
  float deg2 = counts2 * (360.0 / COUNTS_PER_REV);

  Serial.print("Enc1 (deg): ");
  Serial.print(deg1, 2);
  Serial.print("    Enc2 (deg): ");
  Serial.println(deg2, 2);

  delay(5);
}