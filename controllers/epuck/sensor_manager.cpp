#include "sensor_manager.hpp"
#include <iostream>
#include <cstdio>

SensorManager::SensorManager()
    : historyIndex(0), isBaselineInitialized(false)
{
    // Initialize arrays
    for (int i = 0; i < Config::NUM_SENSORS; i++)
    {
        baselineValues[i] = 0;
        for (int j = 0; j < Config::HISTORY_SIZE; j++)
        {
            sensorHistory[i][j] = 0;
        }
    }
}

SensorManager::~SensorManager()
{
    // Cleanup if needed
}

void SensorManager::initializeSensors(webots::Robot *robot)
{
    char sensorName[5];
    for (int i = 0; i < Config::NUM_SENSORS; i++)
    {
        sprintf(sensorName, "ls%d", i);
        lightSensors[i] = robot->getLightSensor(sensorName);
        lightSensors[i]->enable(Config::TIME_STEP);
    }
}

void SensorManager::calibrateSensors(webots::Robot *robot)
{
    double tempBaseline[Config::NUM_SENSORS] = {0};

    // Initial step to ensure sensors are ready
    robot->step(Config::TIME_STEP);

    // Collect samples
    for (int sample = 0; sample < Config::CALIBRATION_SAMPLES; sample++)
    {
        for (int i = 0; i < Config::NUM_SENSORS; i++)
        {
            tempBaseline[i] += lightSensors[i]->getValue();
        }
        robot->step(Config::TIME_STEP);
    }

    // Calculate baselines and initialize history
    for (int i = 0; i < Config::NUM_SENSORS; i++)
    {
        baselineValues[i] = tempBaseline[i] / Config::CALIBRATION_SAMPLES;
        for (int j = 0; j < Config::HISTORY_SIZE; j++)
        {
            sensorHistory[i][j] = baselineValues[i];
        }
    }

    isBaselineInitialized = true;
}

float SensorManager::readSensors(double *sensorValues)
{

    std::cout << "Sensor values: ";
    for (int i = 0; i < Config::NUM_SENSORS; i++)
    {
        double rawValue = lightSensors[i]->getValue();
        updateSensorHistory(i, rawValue);

        double smoothedValue = calculateSmoothedValue(i);
        double processedValue = smoothedValue - baselineValues[i];
        processedValue = (processedValue < 0) ? 0 : processedValue;

        sensorValues[i] = abs(processedValue - sensorValues[i]) >
                                  Config::STABILIZING_THRESHOLD
                              ? processedValue
                              : sensorValues[i];

        std::cout << i << ":" << rawValue << " ";
    }
    std::cout << std::endl;

    historyIndex = (historyIndex + 1) % Config::HISTORY_SIZE;

    float error = 0;
    // float right_error = SIDE_DISTANCE - right_tof;
    // int left_error = SIDE_DISTANCE - left_tof;
    // if (g_steering_mode == STEER_NORMAL)
    // {
    //     if (sensors.leftWallExist && sensors.rightWallExist)
    //     {
    //         error = left_error - right_error;
    //     }
    //     else if (sensors.leftWallExist)
    //     {
    //         error = 2 * left_error;
    //     }
    //     else if (sensors.rightWallExist)
    //     {
    //         error = -2 * right_error;
    //     }
    // }
    // m_cross_track_error = error;

    return error;
}

void SensorManager::updateSensorHistory(int sensorIndex, double value)
{
    sensorHistory[sensorIndex][historyIndex] = value;
}

double SensorManager::calculateSmoothedValue(int sensorIndex)
{
    double sum = 0;
    for (int j = 0; j < Config::HISTORY_SIZE; j++)
    {
        sum += sensorHistory[sensorIndex][j];
    }
    return sum / Config::HISTORY_SIZE;
}