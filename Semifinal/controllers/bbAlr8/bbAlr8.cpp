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

    // while (step(Config::TIME_STEP) != -1)
    // {
    //     seeSurvivors();
    // }

    motors.enterMaze(this, sensorManager);

    std::cout << "inside the maze" << std::endl;

    std::array<std::array<unsigned int, MAZE_SIZE>, MAZE_SIZE> searchedMaze = exploreMaze();

    API_turnRight();
    motors.moveForward(this, sensorManager, 1.5);
    API_turn180();

    std::cout << "Back at start" << std::endl;

    // std::array<std::array<unsigned int, MAZE_SIZE>, MAZE_SIZE> searchedMaze = {{{3, 13, 3, 13, 3, 5, 5, 5, 1, 1, 9, 7, 5, 9, 3, 1, 5, 5, 1, 9},
    //                                                                             {2, 5, 8, 3, 12, 3, 1, 1, 4, 8, 6, 5, 5, 12, 10, 14, 3, 1, 8, 10},
    //                                                                             {10, 7, 4, 12, 3, 12, 14, 10, 11, 6, 5, 5, 5, 5, 12, 3, 12, 10, 10, 10},
    //                                                                             {10, 3, 1, 9, 10, 3, 5, 4, 12, 3, 9, 3, 5, 5, 9, 10, 3, 12, 10, 10},
    //                                                                             {6, 12, 10, 10, 10, 10, 7, 9, 3, 12, 10, 6, 5, 9, 2, 8, 10, 7, 12, 10},
    //                                                                             {3, 5, 12, 6, 8, 6, 9, 2, 12, 11, 6, 9, 3, 12, 2, 12, 10, 3, 9, 10},
    //                                                                             {6, 13, 3, 13, 6, 1, 0, 8, 3, 0, 13, 10, 6, 9, 14, 7, 4, 8, 6, 8},
    //                                                                             {3, 9, 2, 1, 9, 2, 4, 4, 8, 14, 3, 12, 11, 2, 9, 3, 5, 12, 3, 8},
    //                                                                             {10, 6, 4, 8, 10, 2, 1, 9, 6, 1, 0, 5, 12, 10, 10, 6, 5, 9, 6, 8},
    //                                                                             {10, 3, 9, 10, 10, 2, 8, 6, 5, 4, 4, 9, 11, 10, 6, 5, 13, 10, 3, 12},
    //                                                                             {14, 10, 6, 12, 6, 12, 10, 3, 5, 5, 1, 12, 6, 12, 3, 5, 5, 12, 2, 9},
    //                                                                             {11, 6, 5, 1, 1, 9, 10, 6, 1, 13, 6, 1, 5, 1, 12, 3, 9, 11, 6, 8},
    //                                                                             {2, 9, 3, 12, 14, 10, 14, 3, 12, 3, 9, 10, 3, 8, 3, 12, 6, 8, 3, 12},
    //                                                                             {10, 6, 4, 13, 3, 8, 3, 0, 13, 10, 10, 2, 4, 12, 2, 1, 1, 4, 8, 11},
    //                                                                             {6, 9, 7, 1, 12, 6, 4, 4, 1, 8, 10, 2, 13, 3, 8, 6, 0, 1, 12, 10},
    //                                                                             {3, 4, 9, 2, 5, 9, 7, 5, 4, 8, 10, 6, 5, 0, 4, 1, 12, 6, 5, 8},
    //                                                                             {2, 9, 6, 12, 3, 12, 7, 1, 1, 8, 6, 9, 3, 4, 1, 8, 3, 1, 9, 10},
    //                                                                             {10, 6, 5, 13, 2, 1, 1, 12, 10, 6, 5, 12, 10, 3, 12, 6, 8, 14, 6, 8},
    //                                                                             {2, 5, 9, 3, 12, 2, 12, 11, 10, 3, 13, 3, 8, 6, 9, 11, 10, 3, 9, 10},
    //                                                                             {6, 13, 6, 12, 7, 12, 7, 4, 4, 4, 5, 12, 6, 5, 12, 6, 12, 14, 6, 12}}};


    rescueAlgo.setMaze(searchedMaze); //instance is already created. don't add here as it will redirect new colors and survivor arrays
    std::cout << "Maze converted" << std::endl;

    std::cout << "Start Rescuing in " << std::endl;

    leds.lightEachLEDSequentially(*this);

    std::cout << "BB-ALr-8 is on the move. Timers ticking" << std::endl;
    auto startTime = std::chrono::high_resolution_clock::now();

    motors.enterMaze(this, sensorManager);

    rescueAlgo.findOptimalRoute();

    // Follow the calculated path
    if (!rescueAlgo.hasPathCalculated())
        return;
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
            case RescueRunAlgo::Command::WAIT:
                std::cout << "Found survivor! Waiting for 3 seconds..." << std::endl;

                leds.lightGreenOn();
                // Delay for 3 seconds
                int delaySteps = (Config::DELAY_TIME / Config::TIME_STEP);
                for (int i = 0; i < delaySteps; i++)
                {
                    step(Config::TIME_STEP);
                }
                leds.lightGreenOff();
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

    API_turnRight();
    motors.moveForward(this, sensorManager, 1.5);
    API_turn180();

    auto endTime = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);

    // Convert to minutes and seconds
    long long totalSeconds = duration.count() / 1000;
    int minutes = totalSeconds / 60;
    int seconds = totalSeconds % 60;
    int milliseconds = duration.count() % 1000;

    // Print the timing results
    std::cout << "\n=== Rescue Operation Complete ===" << std::endl;
    std::cout << "Total time taken: " << minutes << " minutes, "
              << seconds << " seconds, "
              << milliseconds << " milliseconds" << std::endl;
    std::cout << "(Total milliseconds: " << duration.count() << ")" << std::endl;
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
    if (!isRedNode(x, y))
    {
        rescueAlgo.redNodes.push_back({y, x});
    }

    std::cout << "Updated redNodes: ";
    for (const auto &node : rescueAlgo.redNodes)
    {
        std::cout << "(" << node.x << ", " << node.y << ") ";
    }
    std::cout << std::endl;
}

void BbAlr8::addOrangeNode(int x, int y)
{
    if (!isOrangeNode(x, y) && !isRedNode(x, y))
    {
        rescueAlgo.orangeNodes.push_back({y, x});
    }

    std::cout << "Updated orangeNodes: ";
    for (const auto &node : rescueAlgo.orangeNodes)
    {
        std::cout << "(" << node.x << ", " << node.y << ") ";
    }
    std::cout << std::endl;
}

void BbAlr8::addYellowNode(int x, int y)
{
    if (!isYellowNode(x, y) && !isOrangeNode(x, y) && !isRedNode(x, y))
    {
        rescueAlgo.yellowNodes.push_back({y, x});
    }

    std::cout << "Updated yellowNodes: ";
    for (const auto &node : rescueAlgo.yellowNodes)
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
bool BbAlr8::isYellowNode(int x, int y) const
{
    Point p = {y, x}; // Note: Point format is {y, x}
    return std::find(rescueAlgo.yellowNodes.begin(), rescueAlgo.yellowNodes.end(), p) != rescueAlgo.yellowNodes.end();
}

bool BbAlr8::isSurvivorNode(int x, int y) const
{
    Point p = {y, x}; // Note: Point format is {y, x}
    return std::find(rescueAlgo.survivors.begin(), rescueAlgo.survivors.end(), p) != rescueAlgo.survivors.end();
}

bool BbAlr8::seeSurvivors()
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

void BbAlr8::detectAndAddSurvivors(int x, int y)
{

    if (seeSurvivors())
    {
        if (!isSurvivorAdded(x, y))
        {
            rescueAlgo.survivors.push_back({y, x});
        }

        std::cout << "Updated survivor List: ";
        for (const auto &node : rescueAlgo.survivors)
        {
            std::cout << "(" << node.x << ", " << node.y << ") ";
        }
        std::cout << std::endl;
    }
}

bool BbAlr8::isSurvivorAdded(int x, int y) const
{
    Point p = {y, x}; // Note: Point format is {y, x}
    return std::find(rescueAlgo.survivors.begin(), rescueAlgo.survivors.end(), p) != rescueAlgo.survivors.end();
}