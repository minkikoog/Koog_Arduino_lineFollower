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

// Hysteresis threshold
#define THRESHOLD 400
#define HYSTERESIS 50

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
  analogWrite(ENA, 30); // Motor 1 speed
  analogWrite(ENB, 30); // Motor 2 speed
}

void loop() {
  int leftSensorValue = analogRead(lefts);
  int rightSensorValue = analogRead(rights);

  // There is reflected light from the surface (no line)
  if (leftSensorValue <= (THRESHOLD - HYSTERESIS) && rightSensorValue <= (THRESHOLD - HYSTERESIS)) {
    // Go forward
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }
  // Line detected by left sensor (no light reflected to left sensor), robot turns left
  else if (leftSensorValue > (THRESHOLD + HYSTERESIS) && rightSensorValue <= (THRESHOLD - HYSTERESIS)) {
    // Turn left
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }
  // Line detected by right sensor (no light reflected to right sensor), robot turns right
  else if (leftSensorValue <= (THRESHOLD - HYSTERESIS) && rightSensorValue > (THRESHOLD + HYSTERESIS)) {
    // Turn right
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }
  // Both sensors receive no reflected light
  else if (leftSensorValue > (THRESHOLD + HYSTERESIS) && rightSensorValue > (THRESHOLD + HYSTERESIS)) {
    // Stop
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }
}
