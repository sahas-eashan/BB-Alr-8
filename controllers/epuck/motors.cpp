#include "motors.hpp"
#include "math.h"

Motors::Motors() : 
    leftMotor(nullptr), 
    rightMotor(nullptr),
    previousLeftPosition(0),
    previousRightPosition(0),
    distanceTraveled(0)
{}

Motors::~Motors() {}

void Motors::initializeMotors(webots::Robot* robot) {
    leftMotor = robot->getMotor("left wheel motor");
    rightMotor = robot->getMotor("right wheel motor");
    
    // Initialize motors for velocity control
    leftMotor->setPosition(INFINITY);
    rightMotor->setPosition(INFINITY);
    
    // Start with motors stopped
    setSpeed(0.0, 0.0);
    
}

void Motors::setSpeed(double leftSpeed, double rightSpeed) {
    // Ensure speeds are within bounds
    leftSpeed = clamp(leftSpeed, -Config::MAX_SPEED, Config::MAX_SPEED);
    rightSpeed = clamp(rightSpeed, -Config::MAX_SPEED, Config::MAX_SPEED);
    
    leftMotor->setVelocity(leftSpeed);
    rightMotor->setVelocity(rightSpeed);
}

void Motors::stop() {
    setSpeed(0.0, 0.0);
}

void Motors::moveForward(double distance) {
    double radians = convertDistanceToRadians(distance);
    double currentLeft = leftMotor->getPositionSensor()->getValue();
    double currentRight = rightMotor->getPositionSensor()->getValue();
    
    setPosition(currentLeft + radians, currentRight + radians);
    waitForMotorsToStop();
    
    // Update distance traveled
    distanceTraveled += distance;
}

void Motors::turnRight() {
    double turnRadians = (M_PI / 2.0) * (Config::WHEEL_DISTANCE / Config::WHEEL_RADIUS);
    double currentLeft = leftMotor->getPositionSensor()->getValue();
    double currentRight = rightMotor->getPositionSensor()->getValue();
    
    setPosition(currentLeft + turnRadians, currentRight - turnRadians);
    waitForMotorsToStop();
}

void Motors::turnLeft() {
    double turnRadians = (M_PI / 2.0) * (Config::WHEEL_DISTANCE / Config::WHEEL_RADIUS);
    double currentLeft = leftMotor->getPositionSensor()->getValue();
    double currentRight = rightMotor->getPositionSensor()->getValue();
    
    setPosition(currentLeft - turnRadians, currentRight + turnRadians);
    waitForMotorsToStop();
}

void Motors::turn180() {
    double turnRadians = M_PI * (Config::WHEEL_DISTANCE / Config::WHEEL_RADIUS);
    double currentLeft = leftMotor->getPositionSensor()->getValue();
    double currentRight = rightMotor->getPositionSensor()->getValue();
    
    setPosition(currentLeft - turnRadians, currentRight + turnRadians);
    waitForMotorsToStop();
}

void Motors::applyPIDCorrection(double correction) {
    double baseSpeed = Config::MAX_SPEED * 0.7;  // Use 70% of max speed for stable control
    double leftSpeed = baseSpeed - correction;
    double rightSpeed = baseSpeed + correction;
    
    setSpeed(leftSpeed, rightSpeed);
}

bool Motors::isMoving() const {
    double leftVel = std::abs(leftMotor->getVelocity());
    double rightVel = std::abs(rightMotor->getVelocity());
    return (leftVel > 0.01 || rightVel > 0.01);
}

double Motors::getDistanceTraveled() const {
    return distanceTraveled;
}

double Motors::getLeftPosition() const {
    return leftMotor->getPositionSensor()->getValue();
}

double Motors::getRightPosition() const {
    return rightMotor->getPositionSensor()->getValue();
}

// Private methods
void Motors::setPosition(double leftPosition, double rightPosition) {
    leftMotor->setPosition(leftPosition);
    rightMotor->setPosition(rightPosition);
    
    // Set velocity for position control
    leftMotor->setVelocity(Config::MAX_SPEED);
    rightMotor->setVelocity(Config::MAX_SPEED);
}

void Motors::waitForMotorsToStop() {
    while (isMoving()) {
        //step(Config::TIME_STEP);
    }
}

double Motors::convertDistanceToRadians(double distance) const {
    return distance / Config::WHEEL_RADIUS;
}

void Motors::resetPositions() {
    previousLeftPosition = getLeftPosition();
    previousRightPosition = getRightPosition();
    distanceTraveled = 0.0;
}

