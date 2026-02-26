#include <Arduino.h>

// =====================================================
// 🧠 ESP32 LINE FOLLOWER — 16 SENSOR (MUX) + TB6612
//     WITH SHARP TURN + DEAD END HANDLING
// =====================================================

// ================= MOTOR PINS =================
#define AIN1 25
#define AIN2 33
#define BIN1 26
#define BIN2 27
#define PWMA 32
#define PWMB 14
#define STBY 18

// ================= MUX PINS =================
#define SIG  34
#define S0   19
#define S1   21
#define S2   22
#define S3   23
#define EN    5

#define SENSOR_COUNT 16

int sensorValues[SENSOR_COUNT];
int sensorMin[SENSOR_COUNT];
int sensorMax[SENSOR_COUNT];
int sensorThreshold[SENSOR_COUNT];

// ================= SPEED =================
#define MAX_SPEED      255
#define BASE_SPEED     60
#define TURN_SPEED     100
#define U_SPEED        65 // 110
#define MIN_SPEED     -255

// ================= PID =================
float Kp = 1.2;
float Ki = 0.0001;
float Kd = 1.2;

int   lastError  = 0;
float totalError = 0;

// ================= LOST LINE =================
#define LOST_THRESHOLD 8
int lostCount = 0;

// ================= DEAD END =================
#define DEAD_END_CONFIRM 15
int deadEndCount = 0;
bool deadEndMode = false;

// ================= LOOP DETECTION =================
#define LOOP_ERROR_THRESHOLD 700
#define LOOP_CONFIRM_COUNT   5

int loopCounter = 0;

// =====================================================
void selectMux(int channel)
{
  digitalWrite(S0,  channel        & 1);
  digitalWrite(S1, (channel >> 1)  & 1);
  digitalWrite(S2, (channel >> 2)  & 1);
  digitalWrite(S3, (channel >> 3)  & 1);
}

// =====================================================
void readSensors()
{
  for (int i = 0; i < SENSOR_COUNT; i++)
  {
    selectMux(i);
    delayMicroseconds(5);
    int val = analogRead(SIG);
    sensorValues[i] = (val < sensorThreshold[i]) ? 1 : 0;
  }
}

// =====================================================
void calibrateSensors()
{
  Serial.println("Calibrating...");
  delay(1000);

  for (int i = 0; i < SENSOR_COUNT; i++) {
    sensorMin[i] = 4095;
    sensorMax[i] = 0;
  }

  unsigned long start = millis();
  while (millis() - start < 5000) {
    for (int i = 0; i < SENSOR_COUNT; i++) {
      selectMux(i);
      delayMicroseconds(5);
      int val = analogRead(SIG);
      if (val < sensorMin[i]) sensorMin[i] = val;
      if (val > sensorMax[i]) sensorMax[i] = val;
    }
  }

  for (int i = 0; i < SENSOR_COUNT; i++)
    sensorThreshold[i] = (sensorMin[i] + sensorMax[i]) / 2;

  Serial.println("Calibration Done!");
}

// =====================================================
int weights[16] = {
  -1000, -850, -700, -550,
  -400,  -250, -120,  -40,
    40,   120,  250,  400,
   550,   700,  850, 1000
};

// =====================================================
int detectSharpTurn()
{
  bool rightExtreme = sensorValues[12] || sensorValues[13] ||
                      sensorValues[14] || sensorValues[15];

  bool leftExtreme  = sensorValues[0] || sensorValues[1] ||
                      sensorValues[2] || sensorValues[3];

  bool centerClear = true;
  for (int i = 4; i <= 11; i++)
    if (sensorValues[i]) { centerClear = false; break; }

  if (centerClear && rightExtreme && !leftExtreme)  return  1;
  if (centerClear && leftExtreme  && !rightExtreme) return -1;

  return 0;
}

// =====================================================
int readLine(int &activeCount)
{
  readSensors();

  long position = 0;
  activeCount = 0;

  for (int i = 0; i < SENSOR_COUNT; i++) {
    if (sensorValues[i]) {
      position += weights[i];
      activeCount++;
    }
  }

  // ALL BLACK → possible dead end
  if (activeCount == SENSOR_COUNT) return 9999;

  if (activeCount == 0) return lastError;

  return (int)(position / activeCount);
}

// =====================================================
void setMotor(int left, int right)
{
  left  = constrain(left,  -255, 255);
  right = constrain(right, -255, 255);

  // LEFT MOTOR
  if (left >= 0) { digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);  }
  else           { digitalWrite(AIN1, LOW);  digitalWrite(AIN2, HIGH); }
  ledcWrite(PWMA, abs(left));

  // RIGHT MOTOR
  if (right >= 0) { digitalWrite(BIN1, LOW);  digitalWrite(BIN2, HIGH); }
  else            { digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);  }
  ledcWrite(PWMB, abs(right));
}

// =====================================================
void setup()
{
  Serial.begin(115200);

  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);

  ledcAttach(PWMA, 1000, 8);
  ledcAttach(PWMB, 1000, 8);

  pinMode(S0, OUTPUT); pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT); pinMode(S3, OUTPUT);
  pinMode(SIG, INPUT);

  pinMode(STBY, OUTPUT); digitalWrite(STBY, HIGH);
  pinMode(EN,   OUTPUT); digitalWrite(EN,   LOW);

  calibrateSensors();
}

// =====================================================
void loop()
{
  int activeCount = 0;
  int error = readLine(activeCount);

  // ================= LOOP DETECTION =================
if (abs(error) > LOOP_ERROR_THRESHOLD && activeCount > 0)
{
  loopCounter++;
}
else
{
  loopCounter = 0;
}

// If stuck turning too long → escape
if (loopCounter > LOOP_CONFIRM_COUNT)
{
  Serial.println("### LOOP DETECTED ###");

  // Stop briefly
  setMotor(0, 0);
  delay(80);

  // Reverse slightly
  setMotor(-BASE_SPEED, -BASE_SPEED);
  delay(250);

  // Force strong opposite turn
  if (error > 0)
    setMotor(-TURN_SPEED, TURN_SPEED);
  else
    setMotor(TURN_SPEED, -TURN_SPEED);

  delay(600);

  loopCounter = 0;
  totalError = 0;

  return;
}

  // ================= LINE LOST RECOVERY =================
  if (activeCount == 0)
  {
    lostCount++;

    if (lostCount >= LOST_THRESHOLD)
    {
      if (lastError > 0)
        setMotor(TURN_SPEED, -TURN_SPEED);
      else
        setMotor(-TURN_SPEED, TURN_SPEED);

      delay(5);
      return;
    }
  }
  else
  {
    lostCount = 0;
  }

  // ================= SHARP TURN =================
  int sharpTurn = detectSharpTurn();

  if (sharpTurn != 0)
  {
    if (sharpTurn == 1)
      setMotor(TURN_SPEED, -TURN_SPEED);
    else
      setMotor(-TURN_SPEED, TURN_SPEED);

    totalError = 0;
    lastError = error;
    delay(5);
    return;
  }

  // ================= DEAD END DETECTION =================
  if (activeCount == 0 || error == 9999)
  {
    deadEndCount++;
    if (deadEndCount >= DEAD_END_CONFIRM)
      deadEndMode = true;
  }
  else
  {
    deadEndCount = 0;
  }

  // ================= DEAD END ACTION =================
  if (deadEndMode)
  {
    Serial.println("### DEAD END ###");

    setMotor(0, 0);
    // delay(100);

    // setMotor(-120, -120);
    // delay(250);

    // if (lastError < 0) // >
      setMotor(U_SPEED, -U_SPEED);
    // else
      // setMotor(-U_SPEED, U_SPEED);

    // delay(450);

    deadEndMode = false;
    deadEndCount = 0;
    totalError = 0;
    return;
  }

  // ================= NORMAL PID =================
  int derivative = error - lastError;
  totalError += error;
  lastError = error;

  float correction = (Kp * error) + (Ki * totalError) + (Kd * derivative);

  int leftSpeed  = BASE_SPEED - correction;
  int rightSpeed = BASE_SPEED + correction;

  leftSpeed  = constrain(leftSpeed,  MIN_SPEED, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, MIN_SPEED, MAX_SPEED);

  setMotor(leftSpeed, rightSpeed);

  delay(5);
}