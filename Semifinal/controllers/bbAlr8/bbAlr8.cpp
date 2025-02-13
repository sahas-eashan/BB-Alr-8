#include "bbAlr8.hpp"
#include <iostream>
#include <chrono>
#include <sstream>
#include <cmath>
#include <thread>
#include <set>

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
    std::cout << "BB-Alr-8 robot starting..." << std::endl;

    motors.enterMaze(this, sensorManager);

    std::cout << "inside the maze" << std::endl;

    exploreMaze();

    rescueAlgo.findOptimalRoute();

    // Follow the calculated path
    if (rescueAlgo.hasPathCalculated())
    {
        const auto &path = rescueAlgo.getOptimalPath();
        Point currentPos = path[0]; // Start position
        int currentHeading = 0;     // Start facing NORTH

        std::set<Point> visitedSurvivors;

        size_t i = 1;
        while (i < path.size())
        {
            Point nextPos = path[i];
            auto movement = rescueAlgo.getNextMovement(currentPos, nextPos, currentHeading);

            // Check if the current position is a survivor's location and hasn't been visited yet
            for (const auto &survivor : rescueAlgo.survivors)
            {
                if (currentPos == survivor && visitedSurvivors.find(survivor) == visitedSurvivors.end())
                {
                    visitedSurvivors.insert(survivor); // Mark survivor as visited

                    leds.lightGreenOn();
                    // Delay for 3 seconds
                    int delaySteps = (Config::DELAY_TIME / Config::TIME_STEP);
                    for (int i = 0; i < delaySteps; i++)
                    {
                        step(Config::TIME_STEP);
                    }
                    leds.lightGreenOff();
                    break; // Exit the loop early since we found a match
                }
            }

            // Execute the movement command
            switch (movement.command)
            {
            case RescueRunAlgo::Command::MOVE_FORWARD:
                std::cout << "Moving forward" << std::endl;
                move_Forward();
                break;
            case RescueRunAlgo::Command::TURN_LEFT:
                std::cout << "Turning left" << std::endl;
                turn_Left();
                currentHeading = (currentHeading + 3) % 4;
                break;
            case RescueRunAlgo::Command::TURN_RIGHT:
                std::cout << "Turning right" << std::endl;
                turn_Right();
                currentHeading = (currentHeading + 1) % 4;
                break;
            case RescueRunAlgo::Command::TURN_180:
                std::cout << "Turning 180" << std::endl;
                turn_180();
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

    // while (step(Config::TIME_STEP) != -1)
    // {
    // leds.lightEachLEDSequentially(*this);
    // floorColor();
    // }
}

int8_t BbAlr8::getFloorColor()
{
    char color = cameraController.processDownCamera();
    auto it = colorMap.find(color);

    if (it != colorMap.end())
    {
        it->second.value != 0 ? std::cout << it->second.name << " Detected" << std::endl : std::cout << " ";
        updateLEDs(it->second.value);
        return it->second.value;
    }

    std::cout << "Unknown Color" << std::endl;
    return -1;
}

bool BbAlr8::see_Survivor()
{
    int green_pixels = cameraController.processScanCamera();

    if (green_pixels > Config::GREEN_PIXEL_COUNT)
    {
        std::cout << std::endl
                  << green_pixels << "  Green pixels detected " << std::endl;
        return true;
    }
    return false;
}

void BbAlr8::updateLEDs(int8_t colorValue)
{
    leds.lightRedOff();
    leds.lightYellowOff();
    leds.lightOrangeOff();

    switch (colorValue)
    {
    case 3:
        leds.lightRedOn();
        break;
    case 2:
        leds.lightRedOn();
        break;
    case 1:
        leds.lightYellowOn();
        break;
    case 4:
        leds.lightRedOn();
        leds.lightYellowOn();
        leds.lightOrangeOn();
        break;
    default:
        break;
    }
}

bool BbAlr8::iswallFront()
{
    sensorManager.readSensors();
    return sensorManager.isWallFront();
}

bool BbAlr8::iswallRight()
{
    sensorManager.readSensors();
    return sensorManager.isWallRight();
}

bool BbAlr8::iswallLeft()
{
    sensorManager.readSensors();
    return sensorManager.isWallLeft();
}

void BbAlr8::move_Forward()
{
    motors.moveForward(this, sensorManager, 1);
}

void BbAlr8::turn_Left()
{
    motors.turnLeft(this);
}

void BbAlr8::turn_Right()
{
    motors.turnRight(this);
}

void BbAlr8::turn_180()
{
    motors.turn180(this);
}

void BbAlr8::addRedNode(int x, int y)
{
    rescueAlgo.redNodes.push_back({y, x});
    std::cout << "Updated redNodes: ";
    for (const auto &node : rescueAlgo.redNodes)
    {
        std::cout << "(" << node.x << ", " << node.y << ") ";
    }
    std::cout << std::endl;
}

void BbAlr8::addOrangeNode(int x, int y)
{
    rescueAlgo.orangeNodes.push_back({y, x});
    std::cout << "Updated orangeNodes: ";
    for (const auto& node : rescueAlgo.orangeNodes)
    {
        std::cout << "(" << node.x << ", " << node.y << ") ";
    }
    std::cout << std::endl;
}

bool BbAlr8::isRedNode(int x, int y) const
{
    Point p = {y, x}; // Note: Point format is {y, x}
    return std::find(rescueAlgo.redNodes.begin(), rescueAlgo.redNodes.end(), p) != rescueAlgo.redNodes.end();
}

bool BbAlr8::isOrangeNode(int x, int y) const
{
    Point p = {y, x}; // Note: Point format is {y, x}
    return std::find(rescueAlgo.orangeNodes.begin(), rescueAlgo.orangeNodes.end(), p) != rescueAlgo.orangeNodes.end();
}