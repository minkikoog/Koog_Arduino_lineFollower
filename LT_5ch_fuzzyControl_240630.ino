#define extraLeftSensor A0
#define leftSensor A1
#define centerSensor A2
#define rightSensor A3
#define extraRightSensor A4

#define ENA 9
#define IN1 7
#define IN2 8
#define ENB 10
#define IN3 11
#define IN4 12

const int numSensors = 5;
const int sensorPins[numSensors] = {A0, A1, A2, A3, A4};

int baseSpeed = 40;
int maxSpeed = 50;

void setup() {
  Serial.begin(9600);

  for (int i = 0; i < numSensors; i++) {
    pinMode(sensorPins[i], INPUT);
  }

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
}

void loop() {
  int sensorValues[numSensors];
  for (int i = 0; i < numSensors; i++) {
    sensorValues[i] = analogRead(sensorPins[i]);
  }

  float fuzzyOutput = fuzzyControl(sensorValues);
  
  int leftMotorSpeed = baseSpeed - fuzzyOutput;
  int rightMotorSpeed = baseSpeed + fuzzyOutput;

  leftMotorSpeed = constrain(leftMotorSpeed, -maxSpeed, maxSpeed);
  rightMotorSpeed = constrain(rightMotorSpeed, -maxSpeed, maxSpeed);

  setMotorSpeed(leftMotorSpeed, rightMotorSpeed);

  Serial.print("Fuzzy Output: ");
  Serial.print(fuzzyOutput);
  Serial.print("\tLeft Speed: ");
  Serial.print(leftMotorSpeed);
  Serial.print("\tRight Speed: ");
  Serial.println(rightMotorSpeed);

  delay(10);
}

float fuzzyControl(int sensorValues[]) {
  float membership[5] = {0, 0, 0, 0, 0};
  float output = 0;

  // 퍼지화 (Fuzzification)
  for (int i = 0; i < numSensors; i++) {
    if (sensorValues[i] < 100) membership[i] = 1;
    else if (sensorValues[i] < 900) membership[i] = (900 - sensorValues[i]) / 800.0;
    else membership[i] = 0;
  }

  // 규칙 평가 및 비퍼지화 (Rule evaluation and Defuzzification)
  float sumMembership = 0;
  float weights[5] = {-1.0, -0.5, 0, 0.5, 1.0};
  for (int i = 0; i < numSensors; i++) {
    sumMembership += membership[i];
    output += membership[i] * weights[i] * 100; // -100에서 100 사이의 값
  }

  if (sumMembership > 0) {
    output /= sumMembership;
  }

  // 급커브 처리
  if (membership[0] > 0.8 || membership[4] > 0.8) {
    output *= 1.5; // 급커브에서 더 강한 반응
  }

  return output;
}

void setMotorSpeed(int leftSpeed, int rightSpeed) {
  digitalWrite(IN1, leftSpeed > 0 ? HIGH : LOW);
  digitalWrite(IN2, leftSpeed > 0 ? LOW : HIGH);
  analogWrite(ENA, abs(leftSpeed));

  digitalWrite(IN3, rightSpeed > 0 ? HIGH : LOW);
  digitalWrite(IN4, rightSpeed > 0 ? LOW : HIGH);
  analogWrite(ENB, abs(rightSpeed));
}