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

int baseSpeed = 200;
int maxSpeed = 220;

// PID 제어 변수
float Kp = 0.7;
float Ki = 0.01;
float Kd = 0.1;
float error = 0;
float lastError = 0;
float integral = 0;

// 센서 임계값
const int blackThreshold = 700;
const int whiteThreshold = 300;

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

  float pidOutput = pidControl(sensorValues);
  
  int leftMotorSpeed = baseSpeed + pidOutput;  // 변경: - 에서 + 로
  int rightMotorSpeed = baseSpeed - pidOutput; // 변경: + 에서 - 로

  leftMotorSpeed = constrain(leftMotorSpeed, -maxSpeed, maxSpeed);
  rightMotorSpeed = constrain(rightMotorSpeed, -maxSpeed, maxSpeed);

  // 라인을 완전히 벗어났을 때 대응
  if (isOffLine(sensorValues)) {
    handleOffLine();
  } else {
    setMotorSpeed(leftMotorSpeed, rightMotorSpeed);
  }


}

float pidControl(int sensorValues[]) {
  float position = calculatePosition(sensorValues);
  error = position - 2000; // 중앙 위치가 2000

  integral += error;
  integral = constrain(integral, -1000, 1000); // 적분 와인드업 방지

  float derivative = error - lastError;
  
  float output = Kp * error + Ki * integral + Kd * derivative;
  
  lastError = error;
  
  return output;
}

float calculatePosition(int sensorValues[]) {
  float weightedSum = 0;
  int sum = 0;
  
  for (int i = 0; i < numSensors; i++) {
    if (sensorValues[i] > blackThreshold) {
      weightedSum += (i * 1000);
      sum++;
    }
  }
  
  if (sum == 0) {
    return lastError > 0 ? 4000 : 0; // 라인을 벗어났을 때 마지막 방향 유지
  }
  
  return weightedSum / sum;
}

bool isOffLine(int sensorValues[]) {
  for (int i = 0; i < numSensors; i++) {
    if (sensorValues[i] > blackThreshold) {
      return false;
    }
  }
  return true;
}

void handleOffLine() {
  if (lastError > 0) {
    // 오른쪽으로 회전 (변경됨)
    setMotorSpeed(maxSpeed, -maxSpeed);
  } else {
    // 왼쪽으로 회전 (변경됨)
    setMotorSpeed(-maxSpeed, maxSpeed);
  }
}

void setMotorSpeed(int leftSpeed, int rightSpeed) {
  digitalWrite(IN1, leftSpeed > 0 ? HIGH : LOW);
  digitalWrite(IN2, leftSpeed > 0 ? LOW : HIGH);
  analogWrite(ENA, abs(leftSpeed));

  digitalWrite(IN3, rightSpeed > 0 ? HIGH : LOW);
  digitalWrite(IN4, rightSpeed > 0 ? LOW : HIGH);
  analogWrite(ENB, abs(rightSpeed));
}