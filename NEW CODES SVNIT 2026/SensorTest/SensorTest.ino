/*
  LFS-16A Sensor Test Code (ESP32 Version)
  Modified for ESP32 + MUX

  Enable = Active LOW
*/

#include <Arduino.h>

// ================= MUX PINS =================
#define S0 19
#define S1 21
#define S2 22
#define S3 23
#define SIG 34

// ================= ENABLE PIN =================
#define EN 5   // Active LOW

const int numSensors = 16;
int rawSensorValue[16];

// =====================================================
void setup()
{
  Serial.begin(115200);

  // MUX control pins
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);

  // Signal pin
  pinMode(SIG, INPUT);

  // Enable pin (Active LOW)
  pinMode(EN, OUTPUT);
  digitalWrite(EN, LOW);   // 🔥 Enable sensor array

  Serial.println("LFS-16A Sensor Test Started");
}

// =====================================================
void selectChannel(int i)
{
  digitalWrite(S0, i & 0x01);
  digitalWrite(S1, (i >> 1) & 0x01);
  digitalWrite(S2, (i >> 2) & 0x01);
  digitalWrite(S3, (i >> 3) & 0x01);
}

// =====================================================
void read16Sensors()
{
  for (int i = 0; i < numSensors; i++)
  {
    selectChannel(i);
    delayMicroseconds(5);     // small settle time
    rawSensorValue[i] = analogRead(SIG);
  }
}

// =====================================================
void printRawSensorValues()
{
  for (int i = 0; i < numSensors; i++)
  {
    Serial.print("S");
    Serial.print(i);
    Serial.print("=");
    Serial.print(rawSensorValue[i]);
    Serial.print("  ");
  }
  Serial.println();
}

// =====================================================
void loop()
{
  read16Sensors();
  printRawSensorValues();
  delay(300);
}
