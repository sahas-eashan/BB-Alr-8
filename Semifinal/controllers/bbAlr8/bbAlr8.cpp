#include "bbAlr8.hpp"
#include <iostream>
#include <chrono>
#include <sstream>
#include <cmath>
#include <thread>

using namespace webots;

BbAlr8::BbAlr8() : cameraController(this)
{
    initDevices();
}

BbAlr8::~BbAlr8()
{
}

void BbAlr8::initDevices()
{
    leds.initLEDs(*this);
    sensorManager.initializeSensors(this);
    motors.initializeMotors(this);

    // Initialize camera
    cameraController.initializeCameras("fcam", "dcam", "scancam");

    // std::cout << "Motors Initialized" << std::endl;
}

void BbAlr8::run()
{
    std::cout << "E-puck robot starting..." << std::endl;

    motors.enterMaze(this, sensorManager);
    // motors.turnLeft(this);
    // motors.moveForward(this, sensorManager, 3);
    // motors.turnRight(this);
    // motors.moveForward(this, sensorManager, 2);
    // motors.turnRight(this);
    // motors.moveForward(this, sensorManager, 2);
    // motors.turnLeft(this);
    // motors.moveForward(this, sensorManager, 2);
    // motors.turnRight(this);
    // motors.moveForward(this, sensorManager, 2);
    // motors.turnRight(this);
    // motors.moveForward(this, sensorManager, 1);
    std::cout << "inside the maze" << std::endl;

    // exploreMaze();
    // Create rescue algorithm instance and calculate path
    RescueRunAlgo rescueAlgo;
    rescueAlgo.findOptimalRoute();

    // Follow the calculated path
    if (rescueAlgo.hasPathCalculated())
    {
        const auto &path = rescueAlgo.getOptimalPath();
        Point currentPos = path[0]; // Start position
        int currentHeading = 0;     // Start facing NORTH
        size_t i = 1;
        while (i < path.size())
        {
            Point nextPos = path[i];
            auto movement = rescueAlgo.getNextMovement(currentPos, nextPos, currentHeading);

            // Execute the movement command
            switch (movement.command)
            {
            case RescueRunAlgo::Command::MOVE_FORWARD:
                std::cout << "Moving forward" << std::endl;
                moveForward();
                break;
            case RescueRunAlgo::Command::TURN_LEFT:
                std::cout << "Turning left" << std::endl;
                turnLeft();
                currentHeading = (currentHeading + 3) % 4;
                break;
            case RescueRunAlgo::Command::TURN_RIGHT:
                std::cout << "Turning right" << std::endl;
                turnRight();
                currentHeading = (currentHeading + 1) % 4;
                break;
            case RescueRunAlgo::Command::TURN_180:
                std::cout << "Turning 180" << std::endl;
                turn180();
                currentHeading = (currentHeading + 2) % 4;
                break;
            }

            if (movement.command == RescueRunAlgo::Command::MOVE_FORWARD)
            {
                // std::cout << "Next position: " << movement.nextPosition.x << "," << movement.nextPosition.y << std::endl;
                currentPos = nextPos;
                i++;
            }
        }
    }

    while (step(Config::TIME_STEP) != -1)
    {
        // leds.lightEachLEDSequentially(*this);
        // floorColor();
    }

    // API_moveForward();
}

char BbAlr8::floorColor()
{
    char color = cameraController.processDownCamera();
    std::cout << "Detected floor color: " << color << std::endl;
    return color;
}

bool BbAlr8::iswallFront()
{
    sensorManager.readSensors();
    return sensorManager.iswallFront();
}

bool BbAlr8::iswallRight()
{
    sensorManager.readSensors();
    return sensorManager.iswallRight();
}

bool BbAlr8::iswallLeft()
{
    sensorManager.readSensors();
    return sensorManager.iswallLeft();
}

void BbAlr8::moveForward()
{
    motors.moveForward(this, sensorManager, 1);
}

void BbAlr8::turnLeft()
{
    motors.turnLeft(this);
}

void BbAlr8::turnRight()
{
    motors.turnRight(this);
}

void BbAlr8::turn180()
{
    motors.turn180(this);
}