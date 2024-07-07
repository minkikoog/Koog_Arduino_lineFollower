// Sensor pin definitions
#define leftSensor A0
#define leftCenterSensor A1
#define centerSensor A2
#define rightCenterSensor A3
#define rightSensor A4

// Motor pin definitions
#define ENA 9
#define IN1 7
#define IN2 8
#define ENB 10
#define IN3 11
#define IN4 12

// Constants
#define BASE_SPEED 30
#define MAX_SPEED 50
#define THRESHOLD 500  // Adjust this value based on your sensor readings

// PID variables
double Kp = 50, Ki = 0.1, Kd = 10;  // Adjust these values for optimal performance
double error = 0, lastError = 0;
double integral = 0, derivative = 0;
double output = 0;

// Sensor weights (now adjustable)
double leftWeight = 6.0;
double leftCenterWeight = 4.0;
double centerWeight = 0.0;  // Usually 0 as it's the center
double rightCenterWeight = -4.0;
double rightWeight = -6.0;

unsigned long lastTime = 0;
unsigned long sampleTime = 10;  // Sample time in milliseconds

// Variables
int leftValue, leftCenterValue, centerValue, rightCenterValue, rightValue;

void setup() {
  // Set motor control pins as outputs
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  // Set sensor pins as inputs
  pinMode(leftSensor, INPUT);
  pinMode(leftCenterSensor, INPUT);
  pinMode(centerSensor, INPUT);
  pinMode(rightCenterSensor, INPUT);
  pinMode(rightSensor, INPUT);
  
  Serial.begin(9600);
}

void loop() {
  unsigned long now = millis();
  if (now - lastTime >= sampleTime) {
    readSensors();
    calculatePID();
    controlRobot();
    printSensorValues();
    lastTime = now;
  }
}

void readSensors() {
  leftValue = analogRead(leftSensor);
  leftCenterValue = analogRead(leftCenterSensor);
  centerValue = analogRead(centerSensor);
  rightCenterValue = analogRead(rightCenterSensor);
  rightValue = analogRead(rightSensor);
}

void calculatePID() {
  // Calculate the error based on sensor readings with adjustable weights
  error = (leftWeight * leftValue + 
           leftCenterWeight * leftCenterValue + 
           centerWeight * centerValue + 
           rightCenterWeight * rightCenterValue + 
           rightWeight * rightValue) / 400.0;  // Weighted error

  // Calculate PID terms
  integral += error * sampleTime / 1000.0;
  derivative = (error - lastError) / (sampleTime / 1000.0);

  // Calculate output
  output = Kp * error + Ki * integral + Kd * derivative;

  // Limit output to motor speed range
  output = constrain(output, -MAX_SPEED, MAX_SPEED);

  lastError = error;
}

void controlRobot() {
  int leftSpeed = BASE_SPEED - output;
  int rightSpeed = BASE_SPEED + output;
  
  // Ensure speeds are within valid range
  leftSpeed = constrain(leftSpeed, 0, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, 0, MAX_SPEED);
  
  // Set motor directions and speeds
  if (leftSpeed >= 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, leftSpeed);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, -leftSpeed);
  }
  
  if (rightSpeed >= 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, rightSpeed);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, -rightSpeed);
  }
}

void printSensorValues() {
  Serial.print("Left: ");
  Serial.print(leftValue);
  Serial.print("\tLeftCenter: ");
  Serial.print(leftCenterValue);
  Serial.print("\tCenter: ");
  Serial.print(centerValue);
  Serial.print("\tRightCenter: ");
  Serial.print(rightCenterValue);
  Serial.print("\tRight: ");
  Serial.print(rightValue);
  Serial.print("\tError: ");
  Serial.print(error);
  Serial.print("\tOutput: ");
  Serial.println(output);
}

// Function to adjust sensor weights
void adjustSensorWeights(double left, double leftCenter, double center, double rightCenter, double right) {
  leftWeight = left;
  leftCenterWeight = leftCenter;
  centerWeight = center;
  rightCenterWeight = rightCenter;
  rightWeight = right;
}