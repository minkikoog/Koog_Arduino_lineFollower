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

int baseSpeed = 100;
int maxSpeed = 100;

// PID control variables
float Kp = 0.3;
float Ki = 0.001;
float Kd = 100;
float error = 0;
float lastError = 0;
float integral = 0;

// Sensor thresholds
int blackThreshold = 700;
int whiteThreshold = 300;

// Sharp turn threshold
const int sharpTurnThreshold = 30; // Adjust this value to tune sharp turn sensitivity

int sensorMin[numSensors];
int sensorMax[numSensors];

// Timing variables
unsigned long previousMillis = 0;
const long interval = 10; // 10 milliseconds

// Function prototypes
void calibrateSensors();
bool isOffLine(int sensorValues[]);
void handleOffLine();
void setMotorSpeed(int leftSpeed, int rightSpeed, int pidOutput);
float pidControl(int sensorValues[]);
float calculatePosition(int sensorValues[]);

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

  calibrateSensors();
}

void loop() {
  unsigned long currentMillis = millis();
  
  if (currentMillis - previousMillis >= interval) {
    // Save the last time you looped
    previousMillis = currentMillis;

    int sensorValues[numSensors];
    for (int i = 0; i < numSensors; i++) {
      sensorValues[i] = analogRead(sensorPins[i]);
    }

    float pidOutput = pidControl(sensorValues);
    
    int leftMotorSpeed = baseSpeed + pidOutput;
    int rightMotorSpeed = baseSpeed - pidOutput;

    leftMotorSpeed = constrain(leftMotorSpeed, -maxSpeed, maxSpeed);
    rightMotorSpeed = constrain(rightMotorSpeed, -maxSpeed, maxSpeed);

    // Handle off-line case
    if (isOffLine(sensorValues)) {
      handleOffLine();
    } else {
      setMotorSpeed(leftMotorSpeed, rightMotorSpeed, abs(pidOutput));
    }

    Serial.print("PID Output: ");
    Serial.print(pidOutput);
    Serial.print("\tLeft Speed: ");
    Serial.print(leftMotorSpeed);
    Serial.print("\tRight Speed: ");
    Serial.println(rightMotorSpeed);
  }
}

void calibrateSensors() {
  for (int i = 0; i < numSensors; i++) {
    sensorMin[i] = 1023;
    sensorMax[i] = 0;
  }

  unsigned long startTime = millis();
  while (millis() - startTime < 3000) {
    for (int i = 0; i < numSensors; i++) {
      int sensorValue = analogRead(sensorPins[i]);
      if (sensorValue < sensorMin[i]) sensorMin[i] = sensorValue;
      if (sensorValue > sensorMax[i]) sensorMax[i] = sensorValue;
    }
  }

  Serial.println("Calibration results:");
  for (int i = 0; i < numSensors; i++) {
    Serial.print("Sensor ");
    Serial.print(i);
    Serial.print(" min: ");
    Serial.print(sensorMin[i]);
    Serial.print(" max: ");
    Serial.println(sensorMax[i]);
  }
}

float pidControl(int sensorValues[]) {
  float position = calculatePosition(sensorValues);
  error = position - 2000; // Center position is 2000

  integral += error;
  integral = constrain(integral, -1000, 1000); // Prevent integral windup

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
    return lastError > 0 ? 4000 : 0; // Maintain last direction if off-line
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
    // Turn right (sharp turn)
    setMotorSpeed(maxSpeed, -maxSpeed, maxSpeed);
  } else {
    // Turn left (sharp turn)
    setMotorSpeed(-maxSpeed, maxSpeed, maxSpeed);
  }
}

void setMotorSpeed(int leftSpeed, int rightSpeed, int pidOutput) {
  if (pidOutput > sharpTurnThreshold || pidOutput < -sharpTurnThreshold) {
    // Sharp turn: stop or reverse one wheel
    digitalWrite(IN1, leftSpeed > 0 ? HIGH : LOW);
    digitalWrite(IN2, leftSpeed > 0 ? LOW : HIGH);
    analogWrite(ENA, abs(leftSpeed));

    digitalWrite(IN3, rightSpeed > 0 ? HIGH : LOW);
    digitalWrite(IN4, rightSpeed > 0 ? LOW : HIGH);
    analogWrite(ENB, abs(rightSpeed));
  } else {
    // Smooth turn: adjust speeds proportionally
    if (leftSpeed > 0) {
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      analogWrite(ENA, leftSpeed);
    } else {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      analogWrite(ENA, -leftSpeed);
    }

    if (rightSpeed > 0) {
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
      analogWrite(ENB, rightSpeed);
    } else {
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
      analogWrite(ENB, -rightSpeed);
    }
  }
}
