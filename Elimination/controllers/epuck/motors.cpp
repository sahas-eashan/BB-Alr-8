#include "motors.hpp"
#include <thread>


Motors::Motors() {}

Motors::~Motors() {}

void Motors::initializeMotors(webots::Robot *robot)
{
    leftMotor = robot->getMotor("left wheel motor");
    rightMotor = robot->getMotor("right wheel motor");

    leftMotor->setPosition(INFINITY);
    rightMotor->setPosition(INFINITY);

    setSpeed(0.0, 0.0);
}

void Motors::setSpeed(double leftSpeed, double rightSpeed)
{

    leftSpeed = clamp(leftSpeed, -Config::MAX_SPEED, Config::MAX_SPEED);
    rightSpeed = clamp(rightSpeed, -Config::MAX_SPEED, Config::MAX_SPEED);

    leftMotor->setVelocity(leftSpeed);
    rightMotor->setVelocity(rightSpeed);
}

void Motors::stop()
{
    setSpeed(0.0, 0.0);
}



void Motors::delay(int ms)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}