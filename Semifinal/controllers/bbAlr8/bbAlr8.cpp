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
    std::cout << "BB-Alr-8 robot starting..." << std::endl;

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

    exploreMaze();
    // while (step(Config::TIME_STEP) != -1)
    // {
        // leds.lightEachLEDSequentially(*this);
        // floorColor();
    //}

    //API_moveForward(); 
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

void BbAlr8::turn_Left(){
    motors.turnLeft(this);
}

void BbAlr8::turn_Right(){
    motors.turnRight(this);
}

void BbAlr8::turn_180(){
    motors.turn180(this);
}