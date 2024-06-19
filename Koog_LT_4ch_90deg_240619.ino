// Defining pins and variables
#define lefts A0
#define mid_left A1
#define mid_right A2
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
#define NO_LINE_THRESHOLD 100
#define BASE_SPEED 40
#define TURN_SPEED 30
#define SHARP_TURN_SPEED 20
#define TURN_DELAY 1000 // 턴 지속 시간 (밀리초)

// Variables for turn management
bool turning = false;
unsigned long turnStartTime = 0;

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
  pinMode(mid_left, INPUT);
  pinMode(mid_right, INPUT);
  pinMode(rights, INPUT);

  // Setting initial motor speed
  analogWrite(ENA, BASE_SPEED); // Motor 1 speed
  analogWrite(ENB, BASE_SPEED); // Motor 2 speed
}

void loop() {
  // Check if currently turning
  if (turning) {
    // Check if the turn duration has passed
    if (millis() - turnStartTime >= TURN_DELAY) {
      turning = false; // End turning state
    } else {
      // Continue turning (keep motors in turn state)
      return; // Skip sensor reading and motor control
    }
  }

  // Read sensor values
  int leftSensorValue = analogRead(lefts);
  int midLeftSensorValue = analogRead(mid_left);
  int midRightSensorValue = analogRead(mid_right);
  int rightSensorValue = analogRead(rights);

  // Determine motor directions based on sensor values
  if (leftSensorValue > THRESHOLD && midLeftSensorValue > THRESHOLD && midRightSensorValue > THRESHOLD && rightSensorValue > THRESHOLD) {
    // All sensors detect the line - stop
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  } 
  else if (leftSensorValue <= NO_LINE_THRESHOLD && midLeftSensorValue <= NO_LINE_THRESHOLD && midRightSensorValue <= NO_LINE_THRESHOLD && rightSensorValue <= NO_LINE_THRESHOLD) {
    // No sensor detects the line - go forward
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, BASE_SPEED);
    analogWrite(ENB, BASE_SPEED);
  }
  else if (leftSensorValue > THRESHOLD && midLeftSensorValue > THRESHOLD) {
    // Sharp left turn (greater than 90 degrees)
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, SHARP_TURN_SPEED);
    analogWrite(ENB, BASE_SPEED);
    turning = true;
    turnStartTime = millis();
  }
  else if (midLeftSensorValue > THRESHOLD) {
    // Slight left turn
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, BASE_SPEED / 2);
    analogWrite(ENB, BASE_SPEED);
  }
  else if (midRightSensorValue > THRESHOLD) {
    // Slight right turn
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, BASE_SPEED);
    analogWrite(ENB, BASE_SPEED / 2);
  }
  else if (rightSensorValue > THRESHOLD && midRightSensorValue > THRESHOLD) {
    // Sharp right turn (greater than 90 degrees)
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENA, BASE_SPEED);
    analogWrite(ENB, SHARP_TURN_SPEED);
    turning = true;
    turnStartTime = millis();
  }
  else if (rightSensorValue > THRESHOLD) {
    // Sharp right turn
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENA, BASE_SPEED);
    analogWrite(ENB, TURN_SPEED);
  }

  // Small delay for stability
  delay(10);
}
