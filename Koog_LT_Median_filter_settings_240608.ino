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

// Median filter settings
#define NUM_SAMPLES 3

int leftSensorValues[NUM_SAMPLES];
int rightSensorValues[NUM_SAMPLES];
int leftSensorIndex = 0;
int rightSensorIndex = 0;

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
  analogWrite(ENA, 40); // Motor 1 speed
  analogWrite(ENB, 40); // Motor 2 speed

  // Initialize sensor values arrays
  for (int i = 0; i < NUM_SAMPLES; i++) {
    leftSensorValues[i] = analogRead(lefts);
    rightSensorValues[i] = analogRead(rights);
  }
}

// Function to calculate the median of an array
int getMedian(int *values, int numSamples) {
  int sorted[numSamples];
  memcpy(sorted, values, numSamples * sizeof(int));
  sortArray(sorted, numSamples);
  return sorted[numSamples / 2];
}

// Helper function to sort an array
void sortArray(int *arr, int size) {
  for (int i = 0; i < size - 1; i++) {
    for (int j = i + 1; j < size; j++) {
      if (arr[i] > arr[j]) {
        int temp = arr[i];
        arr[i] = arr[j];
        arr[j] = temp;
      }
    }
  }
}

void loop() {
  // Update sensor values
  leftSensorValues[leftSensorIndex] = analogRead(lefts);
  rightSensorValues[rightSensorIndex] = analogRead(rights);
  
  leftSensorIndex = (leftSensorIndex + 1) % NUM_SAMPLES;
  rightSensorIndex = (rightSensorIndex + 1) % NUM_SAMPLES;

  // Calculate median sensor values
  int leftSensorMedian = getMedian(leftSensorValues, NUM_SAMPLES);
  int rightSensorMedian = getMedian(rightSensorValues, NUM_SAMPLES);

  // There is reflected light from the surface (no line)
  if (leftSensorMedian <= 400 && rightSensorMedian <= 400) {
    // Go forward
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }
  // Line detected by left sensor (no light reflected to left sensor), robot turns left
  else if (leftSensorMedian > 400 && rightSensorMedian <= 400) {
    // Turn left
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }
  // Line detected by right sensor (no light reflected to right sensor), robot turns right
  else if (leftSensorMedian <= 400 && rightSensorMedian > 400) {
    // Turn right
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }
  // Both sensors receive no reflected light
  else if (leftSensorMedian > 400 && rightSensorMedian > 400) {
    // Stop
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }

  // Small delay for stability
  delay(10);
}
