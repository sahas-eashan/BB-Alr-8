#ifndef SENSORMANAGER_HPP
#define SENSORMANAGER_HPP

#include <webots/Robot.hpp>
#include <webots/DistanceSensor.hpp>
#include "config.hpp"

class SensorManager {
public:
    SensorManager();
    ~SensorManager();
    
    void initializeSensors(webots::Robot* robot);
    double getDistance(int index) const;
    double getWallError() const;
    double calculateSteeringAdjustment();
    void readSensors(double* sensorValues);

    double frontWallDistance() const;
    double leftWallDistance() const;
    double rightWallDistance() const;


private:
    webots::DistanceSensor* distanceSensors[Config::NUM_SENSORS];
    double distances[Config::NUM_SENSORS] = {0};
    
    // PID control variables
    double previousError = 0;
    double integral = 0;
    
    double calculateSideWallError() const;
    double applyPIDControl(double error);
};
#endif