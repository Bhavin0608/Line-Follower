#include <Arduino.h>

// =====================================================
// 🧠 ESP32 LINE FOLLOWER — 16 SENSOR (MUX) + TB6612
// =====================================================

// ================= MOTOR PINS =================
#define AIN1 25
#define AIN2 33
#define BIN1 26
#define BIN2 27
#define PWMA 32
#define PWMB 14

#define STBY 18   // Optional — connect to HIGH if used

// ================= MUX PINS =================
#define SIG 34
#define S0 19
#define S1 21
#define S2 22
#define S3 23
#define EN 5

#define SENSOR_COUNT 16

int sensorValues[SENSOR_COUNT];
int sensorMin[SENSOR_COUNT];
int sensorMax[SENSOR_COUNT];
int sensorThreshold[SENSOR_COUNT];

// ================= SPEED =================
#define MAX_SPEED 100
#define BASE_SPEED 70
#define MIN_SPEED 0

// ================= PID =================
float Kp = 0.7;
float Ki = 0.0;
float Kd = 0.2;

int lastError = 0;
float totalError = 0;

// =====================================================
// 🔹 SELECT MUX CHANNEL
// =====================================================
void selectMux(int channel)
{
  digitalWrite(S0, channel & 1);
  digitalWrite(S1, (channel >> 1) & 1);
  digitalWrite(S2, (channel >> 2) & 1);
  digitalWrite(S3, (channel >> 3) & 1);
}

// =====================================================
// 🔹 READ ALL 16 SENSORS
// =====================================================
void readSensors()
{
  for (int i = 0; i < SENSOR_COUNT; i++)
  {
    selectMux(i);
    delayMicroseconds(5);

    int val = analogRead(SIG);
    sensorValues[i] = (val < sensorThreshold[i]) ? 1 : 0;

    // Serial.print(val);
    // Serial.print("\n");
  }
}

// =====================================================
// 🔹 CALIBRATION
// =====================================================
void calibrateSensors()
{
  Serial.println("Calibrating 16 sensors...");
  delay(1000);

  for (int i = 0; i < SENSOR_COUNT; i++)
  {
    sensorMin[i] = 4095;
    sensorMax[i] = 0;
  }

  unsigned long start = millis();

  while (millis() - start < 5000)
  {
    for (int i = 0; i < SENSOR_COUNT; i++)
    {
      selectMux(i);
      delayMicroseconds(5);

      int val = analogRead(SIG);

      if (val < sensorMin[i]) sensorMin[i] = val;
      if (val > sensorMax[i]) sensorMax[i] = val;
    }
  }

  for (int i = 0; i < SENSOR_COUNT; i++)
  {
    sensorThreshold[i] =
      (sensorMin[i] + sensorMax[i]) / 2;
  }

  Serial.println("Calibration Done!");
}

// =====================================================
// 🔹 READ LINE POSITION
// =====================================================
int weights[16] = {
  -1000, -850, -700, -550,
  -400,  -250, -120,  -40,
    40,   120,  250,  400,
   550,   700,  850, 1000
};

int readLine()
{
  readSensors();

  long position = 0;
  int active = 0;

  for (int i = 0; i < SENSOR_COUNT; i++)
  {
    if (sensorValues[i])
    {
      position += weights[i];
      active++;
    }
  }

  if (active == 0)
    return lastError;   // Line lost

  int pos = position / active;
  return pos;
}

// =====================================================
// 🔹 MOTOR CONTROL (TB6612)
// =====================================================
void setMotor(int left, int right)
{
  left = constrain(left, -255, 255);
  right = constrain(right, -255, 255);

  // ================= LEFT MOTOR =================
  if (left >= 0)
  {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  }
  else
  {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
  }

  ledcWrite(PWMA, abs(left));

  // ================= RIGHT MOTOR (INVERTED FIX) =================
  if (right >= 0)
  {
    digitalWrite(BIN1, LOW);    // <-- swapped
    digitalWrite(BIN2, HIGH);   // <-- swapped
  }
  else
  {
    digitalWrite(BIN1, HIGH);   // <-- swapped
    digitalWrite(BIN2, LOW);    // <-- swapped
  }

  ledcWrite(PWMB, abs(right));

}


// =====================================================
// 🔹 SETUP
// =====================================================
void setup()
{
  Serial.begin(115200);

  // Motor pins
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  // PWM setup (ESP32)
  ledcAttach(PWMA, 1000, 8);   // pin, freq, resolution
  ledcAttach(PWMB, 1000, 8);


  // MUX pins
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);

  pinMode(SIG, INPUT);

  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  pinMode(EN, OUTPUT);
  digitalWrite(EN, LOW);

  calibrateSensors();
}

// =====================================================
// 🔹 LOOP
// =====================================================
void loop()
{
  int error = readLine();
  Serial.println(error);
  

  // PID
  //-------------- Before ---------------
  // totalError += error;
  // int derivative = error - lastError;

  //-------------- After ----------------
  int derivative = error - lastError;
  totalError += error;
  lastError = error;

  float correction =
    (Kp * error) +
    (Ki * totalError) +
    (Kd * derivative);

  int leftSpeed = BASE_SPEED - correction;
  int rightSpeed = BASE_SPEED + correction;

  leftSpeed = constrain(leftSpeed, MIN_SPEED, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, MIN_SPEED, MAX_SPEED);

  setMotor(leftSpeed, rightSpeed);

  delay(5);
}
// void loop()
// {
//   readSensors();

//   for(int i=0;i<16;i++){
//     Serial.print(sensorValues[i]);
//     Serial.print(" ");
//   }
//   Serial.println();

//   delay(300);
// }

