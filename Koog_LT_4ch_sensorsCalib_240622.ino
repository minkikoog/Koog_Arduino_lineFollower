// Defining pins and variables
#define outside_lefts A0
#define lefts A1
#define rights A2
#define outside_rights A3

// Motor 1 control pins
#define ENA 9
#define IN1 7
#define IN2 8

// Motor 2 control pins
#define ENB 10
#define IN3 11
#define IN4 12

#define THRESHOLD 400
#define BASE_SPEED 30 // Increased base speed
#define MAX_SPEED 150
#define TURN_SPEED 20
#define SHARP_TURN_SPEED 10

// PID controller variables
float Kp = 1.5;  // Adjusted Proportional gain
float Ki = 0.0;  // Integral gain
float Kd = 0.5;  // Derivative gain
float previousError = 0;
float integral = 0;

// Timing variables
unsigned long previousMillis = 0;
const long interval = 10; // Interval for PID calculation

// Sensor calibration variables
int outsideLeftMin = 1023, outsideLeftMax = 0;
int leftMin = 1023, leftMax = 0;
int rightMin = 1023, rightMax = 0;
int outsideRightMin = 1023, outsideRightMax = 0;

// Calibration timing variables
unsigned long calibrationStartTime = 0;
const long calibrationDuration = 5000; // Calibration duration in milliseconds
unsigned long lastCalibrationReadTime = 0;
const long calibrationReadInterval = 50; // Interval between sensor readings during calibration

void setup() {
  // Setting motor control pins as output
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Setting sensor pins as input
  pinMode(outside_lefts, INPUT);
  pinMode(lefts, INPUT);
  pinMode(rights, INPUT);
  pinMode(outside_rights, INPUT);

  // Serial communication for calibration feedback
  Serial.begin(9600);

  // Start calibration
  calibrationStartTime = millis();
}

void loop() {
  unsigned long currentMillis = millis();

  // Calibration phase
  if (currentMillis - calibrationStartTime < calibrationDuration) {
    if (currentMillis - lastCalibrationReadTime >= calibrationReadInterval) {
      lastCalibrationReadTime = currentMillis;
      calibrateSensors();
    }
    return; // Skip the rest of the loop during calibration
  }

  // Normal operation phase
  if (currentMillis - previousMillis >= interval) {
    // Save the last time you ran the PID calculation
    previousMillis = currentMillis;

    // Read sensor values
    int outsideLeftSensorValue = analogRead(outside_lefts);
    int leftSensorValue = analogRead(lefts);
    int rightSensorValue = analogRead(rights);
    int outsideRightSensorValue = analogRead(outside_rights);

    // Map sensor values to calibrated range
    outsideLeftSensorValue = map(outsideLeftSensorValue, outsideLeftMin, outsideLeftMax, 0, 1023);
    leftSensorValue = map(leftSensorValue, leftMin, leftMax, 0, 1023);
    rightSensorValue = map(rightSensorValue, rightMin, rightMax, 0, 1023);
    outsideRightSensorValue = map(outsideRightSensorValue, outsideRightMin, outsideRightMax, 0, 1023);

    // Calculate error
    int error = (outsideLeftSensorValue + leftSensorValue) - (rightSensorValue + outsideRightSensorValue);

    // PID calculations
    integral += error;
    float derivative = error - previousError;
    float correction = Kp * error + Ki * integral + Kd * derivative;
    previousError = error;

    // Calculate motor speeds based on correction
    int speedLeft = BASE_SPEED - correction;
    int speedRight = BASE_SPEED + correction;

    // Constrain motor speeds to be within limits
    speedLeft = constrain(speedLeft, 0, MAX_SPEED);
    speedRight = constrain(speedRight, 0, MAX_SPEED);

    // Apply motor speeds
    analogWrite(ENA, speedLeft);
    analogWrite(ENB, speedRight);

    // Determine motor directions based on sensor values
    if (leftSensorValue <= THRESHOLD && rightSensorValue <= THRESHOLD && outsideLeftSensorValue <= THRESHOLD && outsideRightSensorValue <= THRESHOLD) {
      // Go forward
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
    } else if ((leftSensorValue > THRESHOLD && rightSensorValue <= THRESHOLD) || (outsideLeftSensorValue > THRESHOLD && outsideRightSensorValue <= THRESHOLD)) {
      // Turn left with sharp turn speed
      analogWrite(ENA, SHARP_TURN_SPEED);
      analogWrite(ENB, speedRight);
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
    } else if ((leftSensorValue <= THRESHOLD && rightSensorValue > THRESHOLD) || (outsideLeftSensorValue <= THRESHOLD && outsideRightSensorValue > THRESHOLD)) {
      // Turn right with sharp turn speed
      analogWrite(ENA, speedLeft);
      analogWrite(ENB, SHARP_TURN_SPEED);
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
    } else if ((leftSensorValue > THRESHOLD && rightSensorValue > THRESHOLD) ||
               (outsideLeftSensorValue > THRESHOLD && outsideRightSensorValue > THRESHOLD) ||
               (leftSensorValue > THRESHOLD && outsideRightSensorValue > THRESHOLD) ||
               (outsideLeftSensorValue > THRESHOLD && rightSensorValue > THRESHOLD)) {
      // Stop all motors
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, LOW);

      // Determine turn direction and execute sharp turn
      if (leftSensorValue + outsideLeftSensorValue > rightSensorValue + outsideRightSensorValue) {
        // Sharp left turn
        analogWrite(ENA, SHARP_TURN_SPEED);
        analogWrite(ENB, speedRight);
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
      } else {
        // Sharp right turn
        analogWrite(ENA, speedLeft);
        analogWrite(ENB, SHARP_TURN_SPEED);
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
      }
    } else {
      // Stop
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, LOW);
    }
  }
}

void calibrateSensors() {
  Serial.println("Calibrating sensors...");

  int outsideLeftSensorValue = analogRead(outside_lefts);
  int leftSensorValue = analogRead(lefts);
  int rightSensorValue = analogRead(rights);
  int outsideRightSensorValue = analogRead(outside_rights);

  // Update min and max values
  if (outsideLeftSensorValue < outsideLeftMin) outsideLeftMin = outsideLeftSensorValue;
  if (outsideLeftSensorValue > outsideLeftMax) outsideLeftMax = outsideLeftSensorValue;
  if (leftSensorValue < leftMin) leftMin = leftSensorValue;
  if (leftSensorValue > leftMax) leftMax = leftSensorValue;
  if (rightSensorValue < rightMin) rightMin = rightSensorValue;
  if (rightSensorValue > rightMax) rightMax = rightSensorValue;
  if (outsideRightSensorValue < outsideRightMin) outsideRightMin = outsideRightSensorValue;
  if (outsideRightSensorValue > outsideRightMax) outsideRightMax = outsideRightSensorValue;

  Serial.print("Outside Left Min: "); Serial.println(outsideLeftMin);
  Serial.print("Outside Left Max: "); Serial.println(outsideLeftMax);
  Serial.print("Left Min: "); Serial.println(leftMin);
  Serial.print("Left Max: "); Serial.println(leftMax);
  Serial.print("Right Min: "); Serial.println(rightMin);
  Serial.print("Right Max: "); Serial.println(rightMax);
  Serial.print("Outside Right Min: "); Serial.println(outsideRightMin);
  Serial.print("Outside Right Max: "); Serial.println(outsideRightMax);
}
