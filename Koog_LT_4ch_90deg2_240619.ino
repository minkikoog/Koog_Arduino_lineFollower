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
#define NO_LINE_THRESHOLD 200
#define BASE_SPEED 40
#define TURN_SPEED 30
#define SHARP_TURN_SPEED 20
#define TURN_DELAY 100 // 턴 지속 시간 (밀리초)

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

  // Initialize serial communication for debugging
  Serial.begin(9600);
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

  // Apply filtering (simple moving average for this example)
  static int leftFiltered = leftSensorValue;
  static int midLeftFiltered = midLeftSensorValue;
  static int midRightFiltered = midRightSensorValue;
  static int rightFiltered = rightSensorValue;

  leftFiltered = (leftFiltered + leftSensorValue) / 2;
  midLeftFiltered = (midLeftFiltered + midLeftSensorValue) / 2;
  midRightFiltered = (midRightFiltered + midRightSensorValue) / 2;
  rightFiltered = (rightFiltered + rightSensorValue) / 2;

  // Print sensor values to serial monitor
  Serial.print("Left: ");
  Serial.print(leftFiltered);
  Serial.print("\tMid-Left: ");
  Serial.print(midLeftFiltered);
  Serial.print("\tMid-Right: ");
  Serial.print(midRightFiltered);
  Serial.print("\tRight: ");
  Serial.println(rightFiltered);

  // Determine motor directions based on filtered sensor values
  if (leftFiltered > THRESHOLD && midLeftFiltered > THRESHOLD && midRightFiltered > THRESHOLD && rightFiltered > THRESHOLD) {
    // All sensors detect the line - stop
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  } 
  else if (leftFiltered <= NO_LINE_THRESHOLD && midLeftFiltered <= NO_LINE_THRESHOLD && midRightFiltered <= NO_LINE_THRESHOLD && rightFiltered <= NO_LINE_THRESHOLD) {
    // No sensor detects the line - go forward
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, BASE_SPEED);
    analogWrite(ENB, BASE_SPEED);
  }
  else if (leftFiltered > THRESHOLD && midLeftFiltered > THRESHOLD) {
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
  else if (midLeftFiltered > THRESHOLD) {
    // Slight left turn
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, BASE_SPEED / 2);
    analogWrite(ENB, BASE_SPEED);
  }
  else if (midRightFiltered > THRESHOLD) {
    // Slight right turn
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, BASE_SPEED);
    analogWrite(ENB, BASE_SPEED / 2);
  }
  else if (rightFiltered > THRESHOLD && midRightFiltered > THRESHOLD) {
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
  else if (rightFiltered > THRESHOLD) {
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
