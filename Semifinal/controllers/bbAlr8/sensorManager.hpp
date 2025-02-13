#ifndef SENSORMANAGER_HPP
#define SENSORMANAGER_HPP

#include <webots/Robot.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/PositionSensor.hpp>
#include "config.hpp"

class SensorManager
{
public:
    SensorManager();
    ~SensorManager();

    void initializeSensors(webots::Robot *robot);
    double getWallError() const;
    double calculateSteeringAdjustment();
    void readSensors();

    double frontWallDistance() const; // averaged distance from directly front faced 2 sensors

    // distance value from individiual sensor
    double leftWallDistance() const;
    double rightWallDistance() const;
    double frontRightDistance() const;
    double frontLeftDistance() const;
    double frontRight45AngledDistance() const;
    double frontLeft45AngledDistance() const;

    bool isWallFront();
    bool isWallRight();
    bool isWallLeft();
    int getRightEncoderCount();
    int getLeftEncoderCount();
    
private:
    webots::PositionSensor *rightEncoder;
    webots::PositionSensor *leftEncoder;
    webots::DistanceSensor *distanceSensors[Config::NUM_SENSORS];
    double distances[Config::NUM_SENSORS] = {0};
    double sensorValues[Config::NUM_SENSORS] = {0};

    // PID control variables
    double previousError = 0;
    double integral = 0;

    double calculateSideWallError() const;
    double applyPIDControl(double error);
    double getDistance(int index) const;
};
#endif