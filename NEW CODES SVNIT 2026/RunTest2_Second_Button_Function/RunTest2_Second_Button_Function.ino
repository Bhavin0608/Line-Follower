#include <Arduino.h>

// =====================================================
// 🧠 ESP32 LINE FOLLOWER — LFS-16A + TB6612 + Buttons
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
#define SIG 34
#define S0 19
#define S1 21
#define S2 22
#define S3 23
#define EN 5

// ================= BUTTONS =================
#define BTN_CALIB 13
#define BTN_START 4

// ================= SENSOR =================
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

bool lastCalibState = HIGH;
bool lastStartState = HIGH;

// ================= STATE MACHINE =================
enum RobotState { IDLE, CALIBRATING, RUNNING };
RobotState currentState = IDLE;

// ================= WEIGHTS (Curved Array) =================
int weights[16] = {
  -1000, -850, -700, -550,
  -400,  -250, -120,  -40,
    40,   120,  250,  400,
   550,   700,  850, 1000
};

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
// 🔹 READ SENSORS
// =====================================================
void readSensors()
{
  for (int i = 0; i < SENSOR_COUNT; i++)
  {
    selectMux(i);
    delayMicroseconds(20);

    int val = analogRead(SIG);
    sensorValues[i] = (val < sensorThreshold[i]) ? 1 : 0;
  }
}

// =====================================================
// 🔹 READ LINE POSITION
// =====================================================
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
    return lastError;

  return position / active;
}

// =====================================================
// 🔹 MOTOR CONTROL
// =====================================================
void setMotor(int left, int right)
{
  left = constrain(left, -255, 255);
  right = constrain(right, -255, 255);

  // LEFT MOTOR
  digitalWrite(AIN1, left >= 0);
  digitalWrite(AIN2, left < 0);
  ledcWrite(PWMA, abs(left));

  // RIGHT MOTOR (inverted fix)
  digitalWrite(BIN1, right < 0);
  digitalWrite(BIN2, right >= 0);
  ledcWrite(PWMB, abs(right));
}

// =====================================================
// 🔹 HARD TURN FUNCTIONS
// =====================================================
void hardLeftTurn()
{
  while (true)
  {
    readSensors();
    if (sensorValues[7] || sensorValues[8]) break;
    setMotor(-60, 60);
  }
}

void hardRightTurn()
{
  while (true)
  {
    readSensors();
    if (sensorValues[7] || sensorValues[8]) break;
    setMotor(60, -60);
  }
}

// =====================================================
// 🔹 CALIBRATION WITH SPIN
// =====================================================
void calibrateWithSpin()
{
  for (int i = 0; i < SENSOR_COUNT; i++)
  {
    sensorMin[i] = 4095;
    sensorMax[i] = 0;
  }

  unsigned long start = millis();

  while (millis() - start < 5000)
  {
    setMotor(-100, 100);

    for (int i = 0; i < SENSOR_COUNT; i++)
    {
      selectMux(i);
      delayMicroseconds(20);
      int val = analogRead(SIG);

      if (val < sensorMin[i]) sensorMin[i] = val;
      if (val > sensorMax[i]) sensorMax[i] = val;
    }
  }

  setMotor(0, 0);

  for (int i = 0; i < SENSOR_COUNT; i++)
  {
    sensorThreshold[i] =
      (sensorMin[i] + sensorMax[i]) / 2;
  }

  Serial.println("Calibration Done");
}

// =====================================================
// 🔹 PID RUN FUNCTION
// =====================================================
void runPID()
{
  int error = readLine();

  // Sharp turn detection
  if (error < -800)
  {
    hardLeftTurn();
    return;
  }

  if (error > 800)
  {
    hardRightTurn();
    return;
  }

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
}

// =====================================================
// 🔹 SETUP
// =====================================================
void setup()
{
  Serial.begin(115200);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  ledcAttach(PWMA, 1000, 8);
  ledcAttach(PWMB, 1000, 8);

  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(SIG, INPUT);

  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  pinMode(EN, OUTPUT);
  digitalWrite(EN, LOW);

  pinMode(BTN_CALIB, INPUT_PULLUP);
  pinMode(BTN_START, INPUT_PULLUP);

  Serial.println("Robot Ready - Press CALIB");
}

// =====================================================
// 🔹 LOOP
// =====================================================
void loop()
{
  // -------- CALIB BUTTON --------
  bool calibState = digitalRead(BTN_CALIB);

  if (lastCalibState == HIGH && calibState == LOW && currentState == IDLE)
  {
    delay(50);
    currentState = CALIBRATING;
    Serial.println("Calibration Started");

    calibrateWithSpin();

    currentState = IDLE;
    Serial.println("Waiting for Start");
  }

  lastCalibState = calibState;


  // -------- START BUTTON --------
  bool startState = digitalRead(BTN_START);

  if (lastStartState == HIGH && startState == LOW && currentState == IDLE)
  {
    delay(50);

    totalError = 0;
    lastError = 0;

    currentState = RUNNING;
    Serial.println("Line Following Started");
  }

  lastStartState = startState;


  // -------- RUN MODE --------
  if (currentState == RUNNING)
  {
    runPID();
  }
}
