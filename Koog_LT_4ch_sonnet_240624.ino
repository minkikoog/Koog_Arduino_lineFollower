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
#define BASE_SPEED 30
#define MAX_SPEED 150
#define TURN_SPEED 50
#define SHARP_TURN_SPEED 80

// PID controller variables
float Kp = 2.0;  // Increased Proportional gain
float Ki = 0.1;  // Added small Integral gain
float Kd = 1.0;  // Increased Derivative gain
float previousError = 0;
float integral = 0;

// Timing variables
unsigned long previousMillis = 0;
const long interval = 10;

// Line lost recovery variables
bool lineLost = false;
unsigned long lineLostTime = 0;
const long recoveryTimeout = 1000; // 1 second timeout for recovery

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

  // Setting initial motor speed
  analogWrite(ENA, BASE_SPEED);
  analogWrite(ENB, BASE_SPEED);
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // Read sensor values
    int outsideLeftSensorValue = analogRead(outside_lefts);
    int leftSensorValue = analogRead(lefts);
    int rightSensorValue = analogRead(rights);
    int outsideRightSensorValue = analogRead(outside_rights);

    // Check if line is lost
    if (outsideLeftSensorValue <= THRESHOLD && leftSensorValue <= THRESHOLD &&
        rightSensorValue <= THRESHOLD && outsideRightSensorValue <= THRESHOLD) {
      if (!lineLost) {
        lineLost = true;
        lineLostTime = currentMillis;
      }
    } else {
      lineLost = false;
    }

    // Line recovery mode
    if (lineLost && (currentMillis - lineLostTime < recoveryTimeout)) {
      // Perform a spiral search
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
      analogWrite(ENA, BASE_SPEED);
      analogWrite(ENB, TURN_SPEED);
      return;  // Skip the rest of the loop
    }

    // Calculate error with more weight on outside sensors
    int error = 2 * (outsideLeftSensorValue - outsideRightSensorValue) + (leftSensorValue - rightSensorValue);

    // PID calculations
    integral = integral * 0.8 + error; // Limit integral windup
    float derivative = error - previousError;
    float correction = Kp * error + Ki * integral + Kd * derivative;
    previousError = error;

    // Calculate motor speeds based on correction
    int speedLeft = BASE_SPEED - correction;
    int speedRight = BASE_SPEED + correction;

    // Constrain motor speeds
    speedLeft = constrain(speedLeft, 0, MAX_SPEED);
    speedRight = constrain(speedRight, 0, MAX_SPEED);

    // Determine motor directions and speeds
    if (outsideLeftSensorValue > THRESHOLD || leftSensorValue > THRESHOLD) {
      // Sharp left turn
      speedLeft = SHARP_TURN_SPEED;
      speedRight = MAX_SPEED;
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
    } else if (outsideRightSensorValue > THRESHOLD || rightSensorValue > THRESHOLD) {
      // Sharp right turn
      speedLeft = MAX_SPEED;
      speedRight = SHARP_TURN_SPEED;
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
    } else {
      // Go forward
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
    }

    // Apply motor speeds
    analogWrite(ENA, speedLeft);
    analogWrite(ENB, speedRight);
  }
}
