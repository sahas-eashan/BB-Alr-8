#include "epuck.hpp"
#include <iostream>

using namespace webots;

Epuck::Epuck()
    : pidEnabled(true)
    , lastError(0.0)
    , Kp(Config::DEFAULT_KP)
    , Kd(Config::DEFAULT_KD)
{
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

    // Initialize LEDs
    char ledName[5];
    for (int i = 0; i < Config::NUM_LEDS; i++) {
        sprintf(ledName, "led%d", i);
        leds[i] = getLED(ledName);
    }

    // Initialize sensors
    sensorManager.initializeSensors(this);
}

void Epuck::run() {
    std::cout << "E-puck robot starting..." << std::endl;
    std::cout << "Calibrating sensors..." << std::endl;
    sensorManager.calibrateSensors(this);

    // Main control loop
    while (step(Config::TIME_STEP) != -1) {
        double sensorValues[Config::NUM_SENSORS] = {0};
        sensorManager.readSensors(sensorValues);

        // Obstacle avoidance logic
        double leftSpeed = Config::MAX_SPEED;
        double rightSpeed = Config::MAX_SPEED;

        if (sensorValues[0] > Config::OBSTACLE_THRESHOLD || 
            sensorValues[1] > Config::OBSTACLE_THRESHOLD) {
            leftSpeed = Config::MAX_SPEED;
            rightSpeed = -Config::MAX_SPEED;
        }
        else if (sensorValues[7] > Config::OBSTACLE_THRESHOLD || 
                 sensorValues[6] > Config::OBSTACLE_THRESHOLD) {
            leftSpeed = -Config::MAX_SPEED;
            rightSpeed = Config::MAX_SPEED;
        }

        setMotorSpeeds(leftSpeed, rightSpeed);
    }
}

void Epuck::setMotorSpeeds(double leftSpeed, double rightSpeed) {
    // Ensure speeds don't exceed maximum
    leftSpeed = std::min(std::max(leftSpeed, -Config::MAX_SPEED), Config::MAX_SPEED);
    rightSpeed = std::min(std::max(rightSpeed, -Config::MAX_SPEED), Config::MAX_SPEED);

    // leftMotor->setVelocity(leftSpeed);
    // rightMotor->setVelocity(rightSpeed);
}

void Epuck::togglePID()
{
    pidEnabled = !pidEnabled;

    if (pidEnabled)
    {
        lastError = 0.0;
    }
}

float Epuck::calculatePID(float leftValue, float rightValue)
{
    float error = leftValue - rightValue;

    return error;
}
