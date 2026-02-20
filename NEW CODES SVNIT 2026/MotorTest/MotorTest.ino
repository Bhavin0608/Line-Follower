#include <Arduino.h>

// ===== Motor Driver Pins (TB6612FNG) =====
#define AIN1 25
#define AIN2 33
#define BIN1 26
#define BIN2 27
#define PWMA 32
#define PWMB 14
#define STBY 18   // Enable pin

// =====================================================
void setup()
{
  // Motor direction pins
  Serial.begin(115200);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  // PWM pins
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);

  // Standby pin
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);   // Enable driver
  Serial.println("Motor Test Start");
}

// =====================================================
void loop()
{
  // 🔵 FORWARD
  Serial.println("RUN FORWARD");
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  analogWrite(PWMA, 150);
  analogWrite(PWMB, 150);
  delay(3000);

  // 🔴 BACKWARD
  Serial.println("RUN BACKWARD");
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMA, 150);
  analogWrite(PWMB, 150);
  delay(3000);

  // // 🟢 RIGHT
  Serial.println("RUN RIGHT");
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMA, 150);
  analogWrite(PWMB, 150);
  delay(3000);

  // // 🟡 LEFT
  Serial.println("RUN LEFT");
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  analogWrite(PWMA, 150);
  analogWrite(PWMB, 150);
  delay(3000);

  // // ⚫ STOP
  Serial.println("RUN ENDS");
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
  delay(3000);
}
