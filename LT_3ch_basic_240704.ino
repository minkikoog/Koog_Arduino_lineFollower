// Defining pins and variables
#define lefts A1
#define centers A2
#define rights A3

// Motor 1 control pins
#define ENA 9
#define IN1 7
#define IN2 8

// Motor 2 control pins
#define ENB 10
#define IN3 11
#define IN4 12

#define THRESHOLD 400
#define BASE_SPEED 45
#define MAX_SPEED 125
#define TURN_SPEED 35
#define HARD_TURN_SPEED 50

void setup() {
  // Setting motor control pins as output
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Setting sensor pins as input
  pinMode(lefts, INPUT);
  pinMode(centers, INPUT);
  pinMode(rights, INPUT);

  // Setting initial motor speed
  analogWrite(ENA, BASE_SPEED); // Motor 1 speed
  analogWrite(ENB, BASE_SPEED); // Motor 2 speed
}

void loop() {
  // Read sensor values
  int leftSensorValue = analogRead(lefts);
  int centerSensorValue = analogRead(centers);
  int rightSensorValue = analogRead(rights);

  // Calculate error
  int error = (leftSensorValue - rightSensorValue) / 2; // Adjust error calculation to average left and right sensors

  // Calculate motor speeds based on error
  int speedLeft = BASE_SPEED - error;
  int speedRight = BASE_SPEED + error;

  // Constrain motor speeds to be within limits
  speedLeft = constrain(speedLeft, 0, MAX_SPEED);
  speedRight = constrain(speedRight, 0, MAX_SPEED);

  // Determine motor directions based on sensor values
  if (centerSensorValue < THRESHOLD) { // Center sensor detects line, go forward
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, BASE_SPEED);
    analogWrite(ENB, BASE_SPEED);
  } else if (leftSensorValue < THRESHOLD && centerSensorValue >= THRESHOLD && rightSensorValue >= THRESHOLD) {
    // Left sensor detects line, turn left
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    
    // Determine the turn speed based on sensor value
    if (leftSensorValue < THRESHOLD - 200) {
      analogWrite(ENA, BASE_SPEED);
      analogWrite(ENB, HARD_TURN_SPEED);
    } else {
      analogWrite(ENA, BASE_SPEED);
      analogWrite(ENB, TURN_SPEED);
    }
  } else if (rightSensorValue < THRESHOLD && centerSensorValue >= THRESHOLD && leftSensorValue >= THRESHOLD) {
    // Right sensor detects line, turn right
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    
    // Determine the turn speed based on sensor value
    if (rightSensorValue < THRESHOLD - 200) {
      analogWrite(ENA, HARD_TURN_SPEED);
      analogWrite(ENB, BASE_SPEED);
    } else {
      analogWrite(ENA, TURN_SPEED);
      analogWrite(ENB, BASE_SPEED);
    }
  } else {
    // Stop
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, BASE_SPEED);
    analogWrite(ENB, BASE_SPEED);
  }

  // Small delay for stability
  delay(10);
}

void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}
