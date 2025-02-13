#include "sensorManager.hpp"
#include <iostream>
#include <cstdio>
#include <algorithm>

SensorManager::SensorManager() {}
SensorManager::~SensorManager() {}

void SensorManager::initializeSensors(webots::Robot *robot)
{
    char sensorName[16];
    for (int i = 0; i < Config::NUM_SENSORS; i++)
    {
        sprintf(sensorName, "distanceSensor%d", i);
        distanceSensors[i] = robot->getDistanceSensor(sensorName);
        distanceSensors[i]->enable(Config::TIME_STEP);
    }

    rightEncoder = robot->getPositionSensor("rightEncoder");
    leftEncoder = robot->getPositionSensor("leftEncoder");

    if (rightEncoder)
        rightEncoder->enable(Config::TIME_STEP);
    if (leftEncoder)
        leftEncoder->enable(Config::TIME_STEP);
}

int SensorManager::getRightEncoderCount() {
    return static_cast<int>(rightEncoder->getValue());
}

int SensorManager::getLeftEncoderCount() {
    return static_cast<int>(leftEncoder->getValue());
}

void SensorManager::readSensors()
{
    for (int i = 0; i < Config::NUM_SENSORS; i++)
    {
        // double averagedReading = (distanceSensors[i]->getValue() + sensorValues[i]) / 2; //applying a moving average filter of 2 value window
        double averagedReading = distanceSensors[i]->getValue();
        sensorValues[i] = abs(averagedReading - sensorValues[i]) > Config::STABILIZING_THRESHOLD ? averagedReading : sensorValues[i]; // threshold filtering for stabilizing

        distances[i] = (Config::SENSOR_MAX - sensorValues[i]) /
                       (Config::SENSOR_MAX - Config::SENSOR_MIN) *
                       (Config::MAX_DISTANCE - Config::MIN_DISTANCE);
    }
}

double SensorManager::getDistance(int index) const
{
    if (index >= 0 && index < Config::NUM_SENSORS)
    {
        return distances[index];
    }
    return -1.0; // Invalid index
}

double SensorManager::calculateSideWallError() const
{
    double leftWallDistance = distances[5];
    double rightWallDistance = distances[0];

    // If both walls are detected
    if (leftWallDistance < Config::MAX_WALL_DISTANCE && rightWallDistance < Config::MAX_WALL_DISTANCE)
    {
        return (rightWallDistance - leftWallDistance);
    }
    // If only left wall is detected
    else if (leftWallDistance < Config::MAX_WALL_DISTANCE)
    {
        return 2 * (Config::IDEAL_WALL_DISTANCE - leftWallDistance);
    }
    // If only right wall is detected
    else if (rightWallDistance < Config::MAX_WALL_DISTANCE)
    {
        return 2 * (rightWallDistance - Config::IDEAL_WALL_DISTANCE);
    }

    return 0.0; // No walls detected
}

double SensorManager::getWallError() const
{
    return calculateSideWallError();
}

double SensorManager::applyPIDControl(double error)
{

    double proportional = error;
    integral += error * (Config::TIME_STEP / 1000.0);
    double derivative = (error - previousError) / (Config::TIME_STEP / 1000.0);

    // Anti-windup for integral term
    integral = std::clamp(integral, -Config::MAX_INTEGRAL, Config::MAX_INTEGRAL);

    previousError = error;

    return Config::Kp * proportional + Config::Ki * integral + Config::Kd * derivative;
}

double SensorManager::calculateSteeringAdjustment()
{
    double error = getWallError();

    double adjustment = applyPIDControl(error);

    // Limit the maximum steering adjustment
    return std::clamp(adjustment, -Config::MAX_STEERING, Config::MAX_STEERING);
}

double SensorManager::frontWallDistance() const
{
    // std::cout << (getDistance(1) + getDistance(4))/2 << std::endl;
    return (getDistance(1) + getDistance(4)) / 2;
}

double SensorManager::leftWallDistance() const
{
    return getDistance(5);
}

double SensorManager::rightWallDistance() const
{
    return getDistance(0);
}

double SensorManager::frontRightDistance() const
{
    return getDistance(1);
}

double SensorManager::frontLeftDistance() const
{
    return getDistance(4);
}

double SensorManager::frontRight45AngledDistance() const
{
    return getDistance(2);
}

double SensorManager::frontLeft45AngledDistance() const
{
    return getDistance(3);
}

bool SensorManager::isWallFront()
{
    float F_Wall_Distance = frontWallDistance();

    // std::cout << "Front wall Distance: " << F_Wall_Distance << " cm " << std::endl;
    return (F_Wall_Distance < Config::F_WALL_THRESHOLD) ? true : false;
}

bool SensorManager::isWallRight()
{
    float R_Wall_Distance = rightWallDistance();

    // std::cout << "Right wall Distance: " << R_Wall_Distance << " cm " << std::endl;
    return (R_Wall_Distance < Config::R_WALL_THRESHOLD) ? true : false;
}

bool SensorManager::isWallLeft()
{
    float L_Wall_Distance = leftWallDistance();

    // std::cout << "Left wall Distance: " << L_Wall_Distance << " cm " << std::endl;
    return (L_Wall_Distance < Config::L_WALL_THRESHOLD) ? true : false;
}