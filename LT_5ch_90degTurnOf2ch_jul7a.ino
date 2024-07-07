// Sensor pin definitions
#define leftSensor A0
#define leftCenterSensor A1
#define centerSensor A2
#define rightCenterSensor A3
#define rightSensor A4

// Motor 1 control pins
#define ENA 9
#define IN1 7
#define IN2 8

// Motor 2 control pins
#define ENB 10
#define IN3 11
#define IN4 12

#define THRESHOLD 623 // Adjusted threshold for inverted sensor values
#define BASE_SPEED 45
#define MAX_SPEED 125
#define TURN_SPEED 35
#define HARD_TURN_SPEED 25

void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(leftSensor, INPUT);
  pinMode(leftCenterSensor, INPUT);
  pinMode(centerSensor, INPUT);
  pinMode(rightCenterSensor, INPUT);
  pinMode(rightSensor, INPUT);

  analogWrite(ENA, BASE_SPEED);
  analogWrite(ENB, BASE_SPEED);
}

void loop() {
  int leftValue = 1023 - analogRead(leftSensor);
  int leftCenterValue = 1023 - analogRead(leftCenterSensor);
  int centerValue = 1023 - analogRead(centerSensor);
  int rightCenterValue = 1023 - analogRead(rightCenterSensor);
  int rightValue = 1023 - analogRead(rightSensor);

  int error = (-2 * leftValue - leftCenterValue + rightCenterValue + 2 * rightValue) / 2;

  int speedLeft = BASE_SPEED - error;
  int speedRight = BASE_SPEED + error;

  speedLeft = constrain(speedLeft, 0, MAX_SPEED);
  speedRight = constrain(speedRight, 0, MAX_SPEED);

  if (centerValue > THRESHOLD) {
    // Go forward
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, speedLeft);
    analogWrite(ENB, speedRight);
  } else if (leftValue > THRESHOLD || leftCenterValue > THRESHOLD) {
    // Turn left
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    
    if (leftValue > THRESHOLD + 200) {
      analogWrite(ENA, BASE_SPEED);
      analogWrite(ENB, HARD_TURN_SPEED);
    } else {
      analogWrite(ENA, BASE_SPEED);
      analogWrite(ENB, TURN_SPEED);
    }
  } else if (rightValue > THRESHOLD || rightCenterValue > THRESHOLD) {
    // Turn right
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    
    if (rightValue > THRESHOLD + 200) {
      analogWrite(ENA, HARD_TURN_SPEED);
      analogWrite(ENB, BASE_SPEED);
    } else {
      analogWrite(ENA, TURN_SPEED);
      analogWrite(ENB, BASE_SPEED);
    }
  } else {
    // Go forward
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, speedLeft);
    analogWrite(ENB, speedRight);
  }

  delay(10);
}