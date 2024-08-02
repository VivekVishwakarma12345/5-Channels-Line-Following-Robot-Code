// Define the sensor pins
const int sensorPins[] = {A0, A1, A2, A3, A4};

// Define motor control pins
const int motorLeftForward = 9;
const int motorLeftBackward = 10;
const int motorRightForward = 5;
const int motorRightBackward = 6;

// PID constants
float Kp = 1.0;  // Proportional gain
float Ki = 0.0;  // Integral gain
float Kd = 0.5;  // Derivative gain

// PID variables
float lastError = 0;
float integral = 0;

// Motor speed
int baseSpeed = 150;

void setup() {
    // Initialize sensor pins
    for (int i = 0; i < 5; i++) {
        pinMode(sensorPins[i], INPUT);
    }

    // Initialize motor control pins
    pinMode(motorLeftForward, OUTPUT);
    pinMode(motorLeftBackward, OUTPUT);
    pinMode(motorRightForward, OUTPUT);
    pinMode(motorRightBackward, OUTPUT);

    // Start Serial communication for debugging
    Serial.begin(9600);
}

void loop() {
    int sensorValues[5];
    int position = 0;
    int numSensors = 5;
    
    // Read sensor values
    for (int i = 0; i < numSensors; i++) {
        sensorValues[i] = analogRead(sensorPins[i]);
        if (sensorValues[i] < 500) { // Assuming black line on white surface
            position += (i - 2) * 1000; // Weighted average, -2 to center the position
        }
    }

    int error = position / 1000;
    
    // PID calculations
    int P = error;
    integral += error;
    int I = integral;
    int D = error - lastError;
    lastError = error;

    int correction = Kp * P + Ki * I + Kd * D;

    int leftSpeed = baseSpeed + correction;
    int rightSpeed = baseSpeed - correction;

    // Set motor speeds
    setMotorSpeeds(leftSpeed, rightSpeed);
    
    // Debug output
    Serial.print("Error: ");
    Serial.print(error);
    Serial.print(" Correction: ");
    Serial.println(correction);
    
    delay(50); // Small delay for stability
}

void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    if (leftSpeed > 255) leftSpeed = 255;
    if (leftSpeed < 0) leftSpeed = 0;
    if (rightSpeed > 255) rightSpeed = 255;
    if (rightSpeed < 0) rightSpeed = 0;

    analogWrite(motorLeftForward, leftSpeed);
    analogWrite(motorRightForward, rightSpeed);
}
