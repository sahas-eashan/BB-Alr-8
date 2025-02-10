#include "bbAlr8.hpp"
#include <iostream>
#include <chrono>
#include <sstream>
#include <cmath>
#include <thread>

using namespace webots;

BbAlr8:: BbAlr8() : cameraController(this)
{
    initDevices();
}

BbAlr8:: ~BbAlr8()
{
}

void BbAlr8::initDevices()
{
    leds.initLEDs(*this);
    sensorManager.initializeSensors(this);
    motors.initializeMotors(this);

    // Initialize camera
    cameraController.initializeCameras("fcam", "dcam");

    // std::cout << "Motors Initialized" << std::endl;
}

void BbAlr8::run()
{
    std::cout << "E-puck robot starting..." << std::endl;

    // while (step(Config::TIME_STEP) != -1) {
    //     leds.lightEachLEDSequentially(*this);

    //     sensorManager.readSensors();
    //     std::cout<< "Front: " << sensorManager.frontWallDistance() << "  left: " << sensorManager.leftWallDistance() << "  Right: " << sensorManager.rightWallDistance()  << std::endl;

    //     motors.setSpeed(-10, -10);
    // }

    motors.enterMaze(this, sensorManager);
    motors.turnLeft(this);
    motors.moveForward(this, sensorManager, 4);
    motors.turnRight(this);
    motors.moveForward(this, sensorManager, 2);
}

char BbAlr8::floorColor() {
    char color = cameraController.processDownCamera();
    std::cout << "Detected floor color: " << color << std::endl;
    return color;
}



// 
// const unsigned char* Epuck::getCameraImage() {
//     if (camera) {
//         return camera->getImage();
//     }
//     return nullptr;
// }

// int Epuck::getCameraWidth() const {
//     return camera ? camera->getWidth() : 0;
// }

// int Epuck::getCameraHeight() const {
//     return camera ? camera->getHeight() : 0;
// }