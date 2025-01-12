#ifndef SENSOR_MANAGER_HPP
#define SENSOR_MANAGER_HPP

#include <webots/Robot.hpp>
#include <webots/LightSensor.hpp>
#include "config.hpp"

class SensorManager {
public:
    SensorManager();
    ~SensorManager();

    void initializeSensors(webots::Robot* robot);
    void calibrateSensors(webots::Robot* robot);
    float readSensors(double* sensorValues);
    bool isInitialized() const { return isBaselineInitialized; }

private:
    webots::LightSensor* lightSensors[Config::NUM_SENSORS];
    double sensorHistory[Config::NUM_SENSORS][Config::HISTORY_SIZE];
    double baselineValues[Config::NUM_SENSORS];
    int historyIndex;
    bool isBaselineInitialized;

    void updateSensorHistory(int sensorIndex, double value);
    double calculateSmoothedValue(int sensorIndex);
};

#endif