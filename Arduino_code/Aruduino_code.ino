#include <SPI.h>
#include "BTS7960.h"
#include <AS5X47.h>
#include <Servo.h>

#define CS_PIN 6              // Chip Select pin for AS5147 encoder
#define WHEEL_DIAMETER 0.012  // Diameter of the wheel in meters
#define MAX_PWM 255           // 8-bit max PWM value
#define TARGET_SPEED 15      // Target speed in RPS

// BTS7960 Motor Driver Pins
const uint8_t EN_PIN = 8;
const uint8_t L_PWM_PIN = 5;
const uint8_t R_PWM_PIN = 9;

// Encoder setup
AS5X47 encoder(CS_PIN);

// PID Controller Parameters
float Kp = 0.65, Ki = 0.250, Kd = 0.0;
float integralLimit = 400.0;
float integral = 0;
float previousError = 0;

float currentSpeed = 0.0;
volatile int16_t pidOutput = 0;
unsigned long lastSpeedTime = 0;
unsigned long lastPIDTime = 0;
unsigned long rampUpStartTime = 0;

float lastAngle = 0;
Servo steeringServo;
int servoPin = 3;
int servopwm = 1200;  // Default PWM for neutral steering

bool stopFlag = false;  // Flag to stop the robot
bool startFlag = false; // Flag to start after receiving '1' from Raspberry Pi

void setup() {
    Serial.begin(2000000);
    pinMode(CS_PIN, OUTPUT);
    digitalWrite(CS_PIN, HIGH);
    SPI.begin();
    SPI.setDataMode(SPI_MODE1);
    SPI.setClockDivider(SPI_CLOCK_DIV32);
    steeringServo.attach(servoPin);
    steeringServo.writeMicroseconds(servopwm);
    pinMode(EN_PIN, OUTPUT);
    digitalWrite(EN_PIN, HIGH);

    // Wait for '1' from Raspberry Pi to start the main function
    while (!startFlag) {
        if (Serial.available()) {
            String command = Serial.readStringUntil('\n');
            if (command == "1") {
                startFlag = true;
                stopFlag = false;  // Ensure the robot is not in stop mode
            }
        }
    }

    rampUpStartTime = millis();
}

// Add these new variables at the top:
float totalRPS = 0.0;  // Accumulate RPS
unsigned long lastReportTime = 0;  // Store last reporting time
int speedSampleCount = 0;  // Count of speed samples

void loop() {
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n'); // Read complete command
        int pwmValue = command.toInt(); // Convert string to integer

        if (pwmValue == 0) {
            setBrake(0);
            stopFlag = true;
        } else if (pwmValue >= 1000 && pwmValue <= 1700) {
            servopwm = pwmValue;
            steeringServo.writeMicroseconds(servopwm);
            stopFlag = false;
        } else if (pwmValue == 1) {  // If Raspberry Pi restarts and sends '1'
            stopFlag = false;  // Restart normal operation
        }
    }

    // If stopped, keep checking for a restart signal
    if (stopFlag) {
        while (stopFlag) {
            if (Serial.available()) {
                String command = Serial.readStringUntil('\n');
                if (command == "1") {
                    stopFlag = false; // Exit stop mode
                }
            }
        }
    }

    unsigned long currentTime = millis();

    // Speed calculation every 5ms
    if (currentTime - lastSpeedTime >= 5) {
        calculateSpeedTimer();
        totalRPS += currentSpeed;  // Accumulate RPS
        speedSampleCount++;  // Count the number of samples
        lastSpeedTime = currentTime;
    }

    // PID update every 10ms
    if (currentTime - lastPIDTime >= 10) {
        updatePIDTimer();
        lastPIDTime = currentTime;
    }

    // Send average RPS every 0.5 seconds
    if (currentTime - lastReportTime >= 500) {  // 500ms = 0.5 sec
        float averageRPS = (speedSampleCount > 0) ? (totalRPS / speedSampleCount) : 0.0;  // Avoid division by zero

        Serial.println(averageRPS);  // Send average RPS

        // Reset accumulator and sample count
        totalRPS = 0.0;
        speedSampleCount = 0;
        lastReportTime = currentTime;
    }

    rampUpPWM();
}

float readAngle() {
    float angle = encoder.readAngle();
    angle = fmod(angle, 360.0);
    if (angle < 0) angle += 360.0;
    return angle;
}

void calculateSpeedTimer() {
    unsigned long currentTime = millis();
    float timeChange = (currentTime - lastSpeedTime) / 1000.0;
    if (timeChange <= 0) return;

    float startAngle = lastAngle;
    float endAngle = readAngle();
    float angleChange = endAngle - startAngle;
    if (angleChange < -180.0) angleChange += 360.0;
    if (angleChange > 180.0) angleChange -= 360.0;

    float rotations = abs(angleChange) / 360.0;
    currentSpeed = rotations / timeChange;

    lastAngle = endAngle;
    lastSpeedTime = currentTime;
}

int16_t calculatePID(float targetSpeed, float currentSpeed) {
    float error = targetSpeed - currentSpeed;
    float P = Kp * error;
    integral += error;
    integral = constrain(integral, -integralLimit, integralLimit);
    float I = Ki * integral;
    float output = P + I;
    output = constrain(output, -MAX_PWM, MAX_PWM);
    previousError = error;
    return (int16_t)output;
}

int16_t smoothPIDOutput(int16_t targetPWM) {
    static int16_t lastPWM = 0;
    const int16_t rampRate = 5;
    if (targetPWM > lastPWM) lastPWM = min(lastPWM + rampRate, targetPWM);
    else if (targetPWM < lastPWM) lastPWM = max(lastPWM - rampRate, targetPWM);
    return lastPWM;
}

void updatePIDTimer() {
    pidOutput = calculatePID(TARGET_SPEED, currentSpeed);
    pidOutput = smoothPIDOutput(pidOutput);

    if (pidOutput > 0) {
        setMotorSpeed(pidOutput);
    } else {
        setMotorBrake(-pidOutput);
    }
}

void setMotorSpeed(int16_t pwmValue) {
    if (pwmValue > 0) {
        analogWrite(L_PWM_PIN, pwmValue);
        analogWrite(R_PWM_PIN, 0);
    } else {
        analogWrite(L_PWM_PIN, 0);
        analogWrite(R_PWM_PIN, 0);
    }
}

void setMotorBrake(uint8_t brakeValue) {
    analogWrite(L_PWM_PIN, MAX_PWM - brakeValue);
    analogWrite(R_PWM_PIN, 0);
}

void setBrake(uint8_t brakeValue) {
    analogWrite(L_PWM_PIN, 0);
    analogWrite(R_PWM_PIN, 0);
}

void rampUpPWM() {
    unsigned long currentTime = millis();
    unsigned long rampDuration = 1000;
    int16_t maxStartPWM = 40;

    if (currentTime - rampUpStartTime <= rampDuration) {
        float rampProgress = (float)(currentTime - rampUpStartTime) / rampDuration;
        int16_t rampPWM = rampProgress * maxStartPWM;
        analogWrite(L_PWM_PIN, rampPWM);
        analogWrite(R_PWM_PIN, 0);
    }
}
