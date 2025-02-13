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

    std::cout << "inside the maze" << std::endl;

    exploreMaze();

    // while(step(Config::TIME_STEP) != -1){
    //     see_Survivor();
        //sensorManager.readSensors();
        //std:: cout << " Left : " << sensorManager.leftWallDistance() << "Front : " << sensorManager.frontWallDistance() << " right : "  << sensorManager.rightWallDistance() << std::endl;
    // }
}

int8_t BbAlr8::getFloorColor()
{
    char color = cameraController.processDownCamera();
    auto it = colorMap.find(color);

    if (it != colorMap.end())
    {
        it->second.value != 0? std::cout << it->second.name << " Detected" << std::endl : std::cout << " ";
        updateLEDs(it->second.value);
        return it->second.value;
    }

    std::cout << "Unknown Color" << std::endl;
    return -1;
}

bool BbAlr8::see_Survivor()
{
    int green_pixels = cameraController.processScanCamera();
    
    if (green_pixels > Config::GREEN_PIXEL_COUNT){
        std::cout << std::endl << green_pixels <<  "  Green pixels detected "  << std::endl;
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