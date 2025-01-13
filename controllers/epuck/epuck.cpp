#include "epuck.hpp"
#include <iostream>
#include <chrono>

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

    // Initialize LEDs
    char ledName[5];
    for (int i = 0; i < Config::NUM_LEDS; i++)
    {
        sprintf(ledName, "led%d", i);
        leds[i] = getLED(ledName);
    }

    // Initialize sensors
    sensorManager.initializeSensors(this);
    std::cout << "Sensors Initialized" << std::endl;
    motors.initializeMotors(this);
    std::cout << "Motors Initialized" << std::endl;
}

void Epuck::turnLeft()
{
    motors.setSpeed(-5 ,5);
    step(Config::TIME_90_TURN);
    motors.stop();
}

void Epuck::turnRight()
{
    motors.setSpeed(Config::TURN_SPEED, -Config::TURN_SPEED);
    step(Config::TIME_90_TURN);
    motors.stop();
}

void Epuck::turn180()
{
    motors.setSpeed(Config::TURN_SPEED, -Config::TURN_SPEED);
    step(Config::TIME_90_TURN * 2);
    motors.stop();
}

void Epuck::moveForward(int cells, double *sensorValues)
{
    int totalTime = cells * Config::TIME_PER_CELL;
    auto startTime = std::chrono::steady_clock::now();

    while (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - startTime).count() < totalTime)
    {
        sensorManager.readSensors(sensorValues);
        double correction = sensorManager.calculateSteeringAdjustment();
        motors.setSpeed(Config::BASE_SPEED - correction, Config::BASE_SPEED + correction);

        step(Config::TIME_STEP);
        std::cout << " " << std::endl;
    }

    motors.stop();
}

void Epuck::run()
{
    std::cout << "E-puck robot starting..." << std::endl;

    
    // Main control loop
    while (step(Config::TIME_STEP) != -1)
    {
        sensorManager.readSensors(sensorValues);

        moveForward(4, sensorValues);
        turn180();
        motors.delay(2000);
    }
}