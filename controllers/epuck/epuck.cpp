#include "epuck.hpp"
#include <iostream>

using namespace webots;

Epuck::Epuck() {
    initDevices();
}

Epuck::~Epuck() {
}

void Epuck::initDevices() {
    // Initialize motors
    leftMotor = getMotor("left wheel motor");
    rightMotor = getMotor("right wheel motor");
    leftMotor->setPosition(INFINITY);
    rightMotor->setPosition(INFINITY);
    leftMotor->setVelocity(0.0);
    rightMotor->setVelocity(0.0);
    
    // Initialize distance sensors
    char sensorName[5];
    for (int i = 0; i < NUM_SENSORS; i++) {
        sprintf(sensorName, "ps%d", i);
        distanceSensors[i] = getDistanceSensor(sensorName);
        distanceSensors[i]->enable(TIME_STEP);
    }
    
    // Initialize LEDs
    char ledName[5];
    for (int i = 0; i < NUM_LEDS; i++) {
        sprintf(ledName, "led%d", i);
        leds[i] = getLED(ledName);
    }
}

void Epuck::readSensors(double* sensorValues) {
    for (int i = 0; i < NUM_SENSORS; i++) {
        sensorValues[i] = distanceSensors[i]->getValue();
        // Normalize to 0-1 range
        sensorValues[i] /= 4096.0;
    }
}

void Epuck::setMotorSpeeds(double leftSpeed, double rightSpeed) {
    // Ensure speeds don't exceed maximum
    leftSpeed = std::min(std::max(leftSpeed, -MAX_SPEED), MAX_SPEED);
    rightSpeed = std::min(std::max(rightSpeed, -MAX_SPEED), MAX_SPEED);
    
    leftMotor->setVelocity(leftSpeed);
    rightMotor->setVelocity(rightSpeed);
}

void Epuck::run() {
    std::cout << "E-puck robot starting..." << std::endl;
    
    // Main control loop
    while (step(TIME_STEP) != -1) {
        double sensorValues[NUM_SENSORS];
        readSensors(sensorValues);
        
        // TODO: Add your control logic here
        // Example: Simple obstacle avoidance
        double leftSpeed = MAX_SPEED;
        double rightSpeed = MAX_SPEED;
        
        // If obstacle detected on left side, turn right
        if (sensorValues[0] > 0.5 || sensorValues[1] > 0.5) {
            leftSpeed = MAX_SPEED;
            rightSpeed = -MAX_SPEED;
        }
        // If obstacle detected on right side, turn left
        else if (sensorValues[7] > 0.5 || sensorValues[6] > 0.5) {
            leftSpeed = -MAX_SPEED;
            rightSpeed = MAX_SPEED;
        }
        
        setMotorSpeeds(leftSpeed, rightSpeed);
    }
}