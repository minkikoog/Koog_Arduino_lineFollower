// Defining pins and variables
#define lefts A0 
#define rights A2

// Motor 1 control pins
#define ENA 9
#define IN1 7
#define IN2 8

// Motor 2 control pins
#define ENB 10
#define IN3 11
#define IN4 12

#define THRESHOLD 400
#define BASE_SPEED 90
#define MAX_SPEED 250
#define TURN_SPEED 70

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
  pinMode(rights, INPUT);

  // Setting initial motor speed
  analogWrite(ENA, BASE_SPEED); // Motor 1 speed
  analogWrite(ENB, BASE_SPEED); // Motor 2 speed
}

void loop() {
  // Read sensor values
  int leftSensorValue = analogRead(lefts);
  int rightSensorValue = analogRead(rights);

  // Calculate error
  int error = leftSensorValue - rightSensorValue;

  // Calculate motor speeds based on error
  int speedLeft = BASE_SPEED - error;
  int speedRight = BASE_SPEED + error;

  // Constrain motor speeds to be within limits
  speedLeft = constrain(speedLeft, 0, MAX_SPEED);
  speedRight = constrain(speedRight, 0, MAX_SPEED);

  // Apply motor speeds
  analogWrite(ENA, speedLeft);
  analogWrite(ENB, speedRight);

  // Determine motor directions based on sensor values
  if (leftSensorValue <= THRESHOLD && rightSensorValue <= THRESHOLD) {
    // Go forward
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else if (leftSensorValue > THRESHOLD && rightSensorValue <= THRESHOLD) {
    // Turn left with reduced speed
    analogWrite(ENA, TURN_SPEED);
    analogWrite(ENB, BASE_SPEED);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else if (leftSensorValue <= THRESHOLD && rightSensorValue > THRESHOLD) {
    // Turn right with reduced speed
    analogWrite(ENA, BASE_SPEED);
    analogWrite(ENB, TURN_SPEED);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  } else {
    // Stop
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }

  // Small delay for stability
  delay(10);
}
