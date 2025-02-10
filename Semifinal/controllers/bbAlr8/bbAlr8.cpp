#include "bbAlr8.hpp"
#include <iostream>
#include <chrono>
#include <sstream>
#include <cmath>
#include <thread>

using namespace webots;

BbAlr8:: BbAlr8()
{
    initDevices();
}

BbAlr8:: ~BbAlr8()
{
}

void BbAlr8::initDevices()
{
    leds.initLEDs(*this);

    // Initialize camera
    camera = getCamera("camera");
    if (camera) {
        camera->enable(Config::TIME_STEP);
        std::cout << "Camera enabled with resolution: " 
                  << camera->getWidth() << "x" << camera->getHeight() << std::endl;
    } else {
        std::cerr << "Warning: Camera device not found!" << std::endl;
    }

    // Initialize sensors
    // sensorManager.initializeSensors(this);
    // std::cout << "Sensors Initialized" << std::endl;
    // motors.initializeMotors(this);
    // std::cout << "Motors Initialized" << std::endl;
}

void BbAlr8::run()
{
    std::cout << "E-puck robot starting..." << std::endl;

    while (step(Config::TIME_STEP) != -1) {
        leds.lightEachLEDSequentially(*this);
    }

}

// void Epuck::turnLeft()
// {
//     motors.setSpeed(-5, 5);
//     step(Config::TIME_90_TURN);
//     motors.stop();
// }

// void Epuck::turnRight()
// {
//     motors.setSpeed(Config::TURN_SPEED, -Config::TURN_SPEED);
//     step(Config::TIME_90_TURN);
//     motors.stop();
// }

// void Epuck::turn180()
// {
//     motors.setSpeed(Config::TURN_SPEED, -Config::TURN_SPEED);
//     step(Config::TIME_90_TURN * 2);
//     motors.stop();
// }

// void Epuck::moveForward(int cells, double *sensorValues)
// {
//     int totalTime = cells * Config::TIME_PER_CELL;
//     auto startTime = std::chrono::steady_clock::now();

//     while (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - startTime).count() < totalTime)
//     {
//         sensorManager.readSensors(sensorValues);
//         if (sensorManager.frontWallDistance() < 11){ break; }
//         double correction = sensorManager.calculateSteeringAdjustment();
//         motors.setSpeed(Config::BASE_SPEED - correction, Config::BASE_SPEED + correction);

//         step(Config::TIME_STEP);
//     }

//     if (iswallFront())
//     {
//         while (sensorManager.frontWallDistance() > 11)
//         {
//             sensorManager.readSensors(sensorValues);
//             double correction = sensorManager.calculateSteeringAdjustment();
//             motors.setSpeed(Config::BASE_SPEED - correction, Config::BASE_SPEED + correction);

//             step(Config::TIME_STEP);
//         }
//     }

//     motors.stop();
// }

// bool Epuck::iswallFront()
// {
//     sensorManager.readSensors(sensorValues);
//     float F_Wall_Distance = sensorManager.frontWallDistance();
//     //std::cout << "Front wall Distance: " << F_Wall_Distance << " cm " << std::endl;
//     return (F_Wall_Distance < Config::F_WALL_THRESHOLD) ? true : false;
// }

// bool Epuck::iswallRight()
// {
//     sensorManager.readSensors(sensorValues);
//     float R_Wall_Distance = sensorManager.rightWallDistance();
//     //std::cout << "Right wall Distance: " << R_Wall_Distance << " cm " << std::endl;
//     return (R_Wall_Distance < Config::R_WALL_THRESHOLD) ? true : false;
// }

// bool Epuck::iswallLeft()
// {
//     sensorManager.readSensors(sensorValues);
//     float L_Wall_Distance = sensorManager.leftWallDistance();
//     //std::cout << "Left wall Distance: " << L_Wall_Distance << " cm " << std::endl;
//     return (L_Wall_Distance < Config::L_WALL_THRESHOLD) ? true : false;
// }

// void Epuck::turnToHeading(Config::Heading targetHeading)
// {
//     Node *myEPUCK = getFromDef("EPUCK");
//     if (!myEPUCK)
//     {
//         std::cerr << "Error: Cannot find EPUCK node" << std::endl;
//         return;
//     }

//     Field *rotField = myEPUCK->getField("rotation");
//     if (!rotField)
//     {
//         std::cerr << "Error: Cannot find rotation field" << std::endl;
//         return;
//     }

//     double targetAngles[4] = {
//         M_PI / 2, // NORTH (90 degrees)
//         0.0,      // EAST (0 degrees)
//         -M_PI / 2,// SOUTH (-90 degrees)
//         M_PI      // WEST (180 degrees)
//     };

    
//     double targetAngle = targetAngles[static_cast<int>(targetHeading)];


//     const double kp = 5.0;      
//     const double threshold = 0.01; 
//     const int maxSteps = 65;  

//     for (int stepCount = 0; stepCount < maxSteps; ++stepCount)
//     {
        
//         const double *rotArray = rotField->getSFRotation();
//         double currentAngle = rotArray[3];

        
//         double error = targetAngle - currentAngle;

//         // Normalize error to range [-π, π]
//         while (error > M_PI)
//             error -= 2.0 * M_PI;
//         while (error < -M_PI)
//             error += 2.0 * M_PI;

        
//         if (std::fabs(error) < threshold)
//         {
//             motors.stop();
//             std::cout << "Turned to heading: " << static_cast<int>(targetHeading)
//                       << ", Final angle: " << currentAngle * 180.0 / M_PI << " degrees" << std::endl;
//             heading = targetHeading; // Update internal state
//             return;
//         }

        
//         double turnSpeed = kp * error; // Apply proportional control
//         turnSpeed = clamp(turnSpeed, -Config::TURN_SPEED, Config::TURN_SPEED); 

        
//         motors.setSpeed(-turnSpeed, turnSpeed);

        
//         step(Config::TIME_STEP);
//     }

    
//     motors.stop();
//     std::cerr << "Failed to turn to heading: " << static_cast<int>(targetHeading) << std::endl;
// }



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