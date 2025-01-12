#include "epuck.hpp"
#include <iostream>

using namespace webots;

Epuck::Epuck()
{
    initDevices();
}

Epuck::~Epuck()
{
}

void Epuck::initDevices()
{
    // Initialize motors
    leftMotor = getMotor("left wheel motor");
    rightMotor = getMotor("right wheel motor");
    leftMotor->setPosition(INFINITY);
    rightMotor->setPosition(INFINITY);
    leftMotor->setVelocity(0.0);
    rightMotor->setVelocity(0.0);

    // Initialize LEDs
    char ledName[5];
    for (int i = 0; i < NUM_LEDS; i++)
    {
        sprintf(ledName, "led%d", i);
        leds[i] = getLED(ledName);
    }

    initSensors();
}

void Epuck::initSensors()
{
    char sensorName[5];
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        sprintf(sensorName, "ps%d", i);
        distanceSensors[i] = getDistanceSensor(sensorName);
        distanceSensors[i]->enable(TIME_STEP);
    }
}

void Epuck::togglePID()
{
    pidEnabled = !pidEnabled;

    if (pidEnabled)
    {
        lastError = 0.0;
    }
}

// to identify baseline value of the sensors
void Epuck::calibrateSensors()
{
    const int NUM_SAMPLES = 50;
    double tempBaseline[NUM_SENSORS] = {0};

    //this is needed to ensure sensors are ready
    step(TIME_STEP);

    // Collect multiple samples
    for (int sample = 0; sample < NUM_SAMPLES; sample++)
    {
        for (int i = 0; i < NUM_SENSORS; i++)
        {
            tempBaseline[i] += distanceSensors[i]->getValue();
        }
        step(TIME_STEP);
    }

    // Average the samples to get baseline
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        baselineValues[i] = tempBaseline[i] / NUM_SAMPLES;
    }

    // Initialize history with baselines
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        for (int j = 0; j < HISTORY_SIZE; j++)
        {
            sensorHistory[i][j] = baselineValues[i];
        }
    }

    isBaselineInitialized = true;
}

void Epuck::readSensors(double *sensorValues)
{
    if (!isBaselineInitialized)
    {
        calibrateSensors();
    }

    std::cout << "Sensor values: ";
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        double rawValue = distanceSensors[i]->getValue();
        
        // Store in circular buffer
        sensorHistory[i][historyIndex] = rawValue;

        // Calculate moving average
        double sum = 0;
        for (int j = 0; j < HISTORY_SIZE; j++)
        {
            sum += sensorHistory[i][j];
        }
        
        double smoothedValue = sum / HISTORY_SIZE;

        // Remove baseline noise and apply threshold
        double processedValue = smoothedValue - baselineValues[i];

        processedValue = (processedValue < 0) ? 0 : processedValue;

        // Store the processed value in the passed array
        sensorValues[i] = abs(processedValue -sensorValues[i]) > STABILIZING_THRESHOLD ? processedValue : sensorValues[i];
        //sensorValues[i] = processedValue;

        std::cout << i << ":" << sensorValues[i] << " ";
    }
    std::cout << std::endl;

    // Update history index
    historyIndex = (historyIndex + 1) % HISTORY_SIZE;
}

float Epuck::calculatePID(float leftValue, float rightValue)
{
    float error = leftValue - rightValue;

    return error;
}

void Epuck::setMotorSpeeds(double leftSpeed, double rightSpeed)
{
    // Ensure speeds don't exceed maximum
    leftSpeed = std::min(std::max(leftSpeed, -MAX_SPEED), MAX_SPEED);
    rightSpeed = std::min(std::max(rightSpeed, -MAX_SPEED), MAX_SPEED);

    leftMotor->setVelocity(0);
    rightMotor->setVelocity(0);
}

void Epuck::run()
{
    std::cout << "E-puck robot starting..." << std::endl;
    std::cout << "Calibarating baselines.." << std::endl;
    calibrateSensors();

    // Main control loop
    while (step(TIME_STEP) != -1)
    {
        double sensorValues[NUM_SENSORS] = {0}; 
        readSensors(sensorValues);

        // TODO: Add your control logic here
        // Example: Simple obstacle avoidance
        double leftSpeed = MAX_SPEED;
        double rightSpeed = MAX_SPEED;

        // If obstacle detected on left side, turn right
        if (sensorValues[0] > 0.5 || sensorValues[1] > 0.5)
        {
            leftSpeed = MAX_SPEED;
            rightSpeed = -MAX_SPEED;
        }
        // If obstacle detected on right side, turn left
        else if (sensorValues[7] > 0.5 || sensorValues[6] > 0.5)
        {
            leftSpeed = -MAX_SPEED;
            rightSpeed = MAX_SPEED;
        }

        setMotorSpeeds(leftSpeed, rightSpeed);
    }
}