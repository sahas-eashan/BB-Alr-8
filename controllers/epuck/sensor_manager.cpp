#include "sensor_manager.hpp"
#include <iostream>
#include <cstdio>

SensorManager::SensorManager() 
    : historyIndex(0), isBaselineInitialized(false) 
{
    // Initialize arrays
    for (int i = 0; i < Config::NUM_SENSORS; i++) {
        baselineValues[i] = 0;
        for (int j = 0; j < Config::HISTORY_SIZE; j++) {
            sensorHistory[i][j] = 0;
        }
    }
}

SensorManager::~SensorManager() {
    // Cleanup if needed
}

void SensorManager::initializeSensors(webots::Robot* robot) {
    char sensorName[5];
    for (int i = 0; i < Config::NUM_SENSORS; i++) {
        sprintf(sensorName, "ps%d", i);
        distanceSensors[i] = robot->getDistanceSensor(sensorName);
        distanceSensors[i]->enable(Config::TIME_STEP);
    }
}

void SensorManager::calibrateSensors(webots::Robot* robot) {
    double tempBaseline[Config::NUM_SENSORS] = {0};

    // Initial step to ensure sensors are ready
    robot->step(Config::TIME_STEP);

    // Collect samples
    for (int sample = 0; sample < Config::CALIBRATION_SAMPLES; sample++) {
        for (int i = 0; i < Config::NUM_SENSORS; i++) {
            tempBaseline[i] += distanceSensors[i]->getValue();
        }
        robot->step(Config::TIME_STEP);
    }

    // Calculate baselines and initialize history
    for (int i = 0; i < Config::NUM_SENSORS; i++) {
        baselineValues[i] = tempBaseline[i] / Config::CALIBRATION_SAMPLES;
        for (int j = 0; j < Config::HISTORY_SIZE; j++) {
            sensorHistory[i][j] = baselineValues[i];
        }
    }

    isBaselineInitialized = true;
}

void SensorManager::readSensors(double* sensorValues) {
    if (!isBaselineInitialized) {
        return;
    }

    std::cout << "Sensor values: ";
    for (int i = 0; i < Config::NUM_SENSORS; i++) {
        double rawValue = distanceSensors[i]->getValue();
        updateSensorHistory(i, rawValue);

        double smoothedValue = calculateSmoothedValue(i);
        double processedValue = smoothedValue - baselineValues[i];
        processedValue = (processedValue < 0) ? 0 : processedValue;

        sensorValues[i] = abs(processedValue - sensorValues[i]) > 
            Config::STABILIZING_THRESHOLD ? processedValue : sensorValues[i];

        std::cout << i << ":" << sensorValues[i] << " ";
    }
    std::cout << std::endl;

    historyIndex = (historyIndex + 1) % Config::HISTORY_SIZE;
}

void SensorManager::updateSensorHistory(int sensorIndex, double value) {
    sensorHistory[sensorIndex][historyIndex] = value;
}

double SensorManager::calculateSmoothedValue(int sensorIndex) {
    double sum = 0;
    for (int j = 0; j < Config::HISTORY_SIZE; j++) {
        sum += sensorHistory[sensorIndex][j];
    }
    return sum / Config::HISTORY_SIZE;
}