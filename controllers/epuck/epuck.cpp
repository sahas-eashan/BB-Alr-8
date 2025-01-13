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

    // Initialize LEDs
    char ledName[5];
    for (int i = 0; i < Config::NUM_LEDS; i++)
    {
        sprintf(ledName, "led%d", i);
        leds[i] = getLED(ledName);
    }

    // Initialize sensors
    sensorManager.initializeSensors(this);
    motors.initializeMotors(this);
}

void Epuck::run()
{
    std::cout << "E-puck robot starting..." << std::endl;

    // Main control loop
    while (step(Config::TIME_STEP) != -1)
    {
        sensorManager.readSensors(sensorValues);

    }
}