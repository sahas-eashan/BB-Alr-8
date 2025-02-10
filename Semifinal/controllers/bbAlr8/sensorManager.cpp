#include "sensorManager.hpp"
#include <iostream>
#include <cstdio>
#include <algorithm>

SensorManager::SensorManager() {}
SensorManager::~SensorManager() {}

void SensorManager::initializeSensors(webots::Robot *robot)
{
    char sensorName[5];
    for (int i = 0; i < Config::NUM_SENSORS; i++)
    {
        sprintf(sensorName, "distanceSensor%d", i);
        distanceSensors[i] = robot->getDistanceSensor(sensorName);
        distanceSensors[i]->enable(Config::TIME_STEP);
    }
}

void SensorManager::readSensors(double *sensorValues)
{
    for (int i = 0; i < Config::NUM_SENSORS; i++)
    {
        double averagedReading = (distanceSensors[i]->getValue() + sensorValues[i]) / 2; //applying a moving average filter of 2 value window 
        sensorValues[i] = abs(averagedReading - sensorValues[i]) > Config::STABILIZING_THRESHOLD ? 
                         averagedReading : sensorValues[i];  // threshold filtering for stabilizing
        
        distances[i] = (Config::SENSOR_MAX - sensorValues[i]) / 
                      (Config::SENSOR_MAX - Config::SENSOR_MIN) * 
                      (Config::MAX_DISTANCE - Config::MIN_DISTANCE);
        
    }
}

double SensorManager::getDistance(int index) const
{
    if (index >= 0 && index < Config::NUM_SENSORS) {
        return distances[index];
    }
    return -1.0; // Invalid index
}

double SensorManager::calculateSideWallError() const
{
    double leftWallDistance = distances[2];  
    double rightWallDistance = distances[5]; 
    
    // If both walls are detected
    if (leftWallDistance < Config::MAX_WALL_DISTANCE && rightWallDistance < Config::MAX_WALL_DISTANCE) {
        return (rightWallDistance - leftWallDistance);
    }
    // If only left wall is detected
    else if (leftWallDistance < Config::MAX_WALL_DISTANCE) {
        return (Config::IDEAL_WALL_DISTANCE - leftWallDistance);
    }
    // If only right wall is detected
    else if (rightWallDistance < Config::MAX_WALL_DISTANCE) {
        return (rightWallDistance - Config::IDEAL_WALL_DISTANCE);
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
    double distanceSensor0 = getDistance(0);
    double distanceSensor7 = getDistance(7);

    return (distanceSensor0*Config::COS10 + distanceSensor7*Config::COS10)/2;
}

double SensorManager::leftWallDistance() const
{
    double distanceSensor5 = getDistance(5);

    return distanceSensor5;
}

double SensorManager::rightWallDistance() const
{
    double distanceSensor2 = getDistance(2);

    return distanceSensor2;
}