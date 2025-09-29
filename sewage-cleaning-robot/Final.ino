#include <AccelStepper.h>

// Constants
const int stepsPerRevolution = 2048;
const int trigPin = 7;
const int echoPin = 6;
const int turbidityPin = A0;
const int controlPin = 2;
const int motorPin1 = 8;
const int motorPin2 = 9;
const int motorPin3 = 10;
const int motorPin4 = 11;
const long triggerDelay = 15000; // 15 seconds
const long actuatorDelay = 10000; // 10 seconds

// Variables
boolean systemOn = false;
boolean triggered = false;
unsigned long triggerStartTime = 0;
AccelStepper myStepper(stepsPerRevolution, motorPin1, motorPin2, motorPin3, motorPin4);

void setup() {
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(turbidityPin, INPUT);
  pinMode(controlPin, OUTPUT); // Control pin as OUTPUT
  digitalWrite(controlPin, LOW); // Initialize control pin to OFF
  delay(actuatorDelay);
  myStepper.setSpeed(15);
}

void loop() {
  long distance = measureDistance();
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  if (distance < 25 && !triggered) {
    digitalWrite(controlPin, HIGH); // Turn ON
    delay(actuatorDelay);            // Actuator delay
    triggered = true;
    triggerStartTime = millis();
    Serial.println("System triggered");
  }

  if (triggered && (millis() - triggerStartTime >= triggerDelay)) {
    digitalWrite(controlPin, LOW);  // Turn OFF
    delay(actuatorDelay);            // Actuator delay
    triggered = false;
    Serial.println("System turned off");
  }

  // Only run the rest of the code if the system is on
  systemOn = digitalRead(controlPin) == HIGH;
  if (systemOn) {
    int turbidityValue = analogRead(turbidityPin);
    float voltage = turbidityValue * (5.0 / 1023.0);
    float estimatedNTU = -0.05 * turbidityValue + 45;
    Serial.print("Turbidity Value: ");
    Serial.print(turbidityValue);
    Serial.print(" (");
    Serial.print(estimatedNTU);
    Serial.println(" NTU)");

    classifyWaste(turbidityValue);

    if (turbidityValue > 300 && turbidityValue < 600) {
      myStepper.moveTo(500);
      delay(10);
      myStepper.moveTo(-500);
      delay(10);
    }
  }

  delay(100);
}

long measureDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH);
  long distance = duration * 0.034 / 2;
  return distance;
}

void classifyWaste(int turbidityValue) {
  const int liquidThreshold = 300;
  const int semiSolidThreshold = 600;

  if (turbidityValue < liquidThreshold) {
    Serial.println("Waste Type: Liquid Waste");
  } else if (turbidityValue < semiSolidThreshold) {
    Serial.println("Waste Type: Semi-Solid Waste");
  } else {
    Serial.println("Waste Type: Solid Waste");
  }
}
