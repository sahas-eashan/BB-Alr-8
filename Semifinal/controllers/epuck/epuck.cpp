#include "epuck.hpp"
#include <iostream>
#include <chrono>
#include <webots/Receiver.hpp>
#include <sstream>
#include <webots/Node.hpp>
#include <webots/Supervisor.hpp>
#include <cmath>
#include <thread>
#include <chrono>

using namespace webots;

Epuck::Epuck() : cameraController(this)
{
    initDevices();
}

Epuck::~Epuck()
{
}

void Epuck::initDevices()
{
    std::cout << "Initializing camera" << std::endl;
    // Initialize camera
    // cameraController.initializeCamera("camera");

    // while (1){
        // char color = floorColor();
    // }

    // Initialize LEDs
    char ledName[5];
    for (int i = 0; i < Config::NUM_LEDS; i++)
    {
        sprintf(ledName, "led%d", i);
        leds[i] = getLED(ledName);
        //
    }

    // Initialize camera
    // cameraController.initializeCamera("camera");

    // while (1){
        // char color = floorColor();
    // }

    // Initialize sensors
    sensorManager.initializeSensors(this);
    std::cout << "Sensors Initialized" << std::endl;
    motors.initializeMotors(this);
    std::cout << "Motors Initialized" << std::endl;
}

Position Epuck::recordOwnPosition()
{
    Node *selfNode = getFromDef("EPUCK"); // Ensure the DEF name matches
    if (selfNode == nullptr)
    {
        std::cerr << "Error: Could not retrieve self node!" << std::endl;
        return {-1, -1}; // Return invalid values in case of an error
    }

    // Get the position of the robot
    const double *position = selfNode->getPosition();
    double x_world = position[0];
    double y_world = position[1];

    // Map to grid coordinates
    int x_mapped = round((x_world + 1.125) * 9 / 2.25);
    int y_mapped = round((y_world + 1.125) * 9 / 2.25);

    // Print world and mapped coordinates
    std::cout << "E-puck world position: x=" << x_world
              << ", y=" << y_world << std::endl;
    std::cout << "Mapped grid coordinates: x=" << x_mapped
              << ", y=" << y_mapped << std::endl;

    // Return the position struct
    return {x_mapped, y_mapped};
}

void Epuck::turnLeft()
{
    motors.setSpeed(-5, 5);
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
        if (sensorManager.frontWallDistance() < 11){ break; }
        double correction = sensorManager.calculateSteeringAdjustment();
        motors.setSpeed(Config::BASE_SPEED - correction, Config::BASE_SPEED + correction);

        step(Config::TIME_STEP);
    }

    if (iswallFront())
    {
        while (sensorManager.frontWallDistance() > 11)
        {
            sensorManager.readSensors(sensorValues);
            double correction = sensorManager.calculateSteeringAdjustment();
            motors.setSpeed(Config::BASE_SPEED - correction, Config::BASE_SPEED + correction);

            step(Config::TIME_STEP);
        }
    }

    motors.stop();
}

bool Epuck::iswallFront()
{
    sensorManager.readSensors(sensorValues);
    float F_Wall_Distance = sensorManager.frontWallDistance();
    //std::cout << "Front wall Distance: " << F_Wall_Distance << " cm " << std::endl;
    return (F_Wall_Distance < Config::F_WALL_THRESHOLD) ? true : false;
}

bool Epuck::iswallRight()
{
    sensorManager.readSensors(sensorValues);
    float R_Wall_Distance = sensorManager.rightWallDistance();
    //std::cout << "Right wall Distance: " << R_Wall_Distance << " cm " << std::endl;
    return (R_Wall_Distance < Config::R_WALL_THRESHOLD) ? true : false;
}

bool Epuck::iswallLeft()
{
    sensorManager.readSensors(sensorValues);
    float L_Wall_Distance = sensorManager.leftWallDistance();
    //std::cout << "Left wall Distance: " << L_Wall_Distance << " cm " << std::endl;
    return (L_Wall_Distance < Config::L_WALL_THRESHOLD) ? true : false;
}



void Epuck::turnToHeading(Config::Heading targetHeading)
{
    Node *myEPUCK = getFromDef("EPUCK");
    if (!myEPUCK)
    {
        std::cerr << "Error: Cannot find EPUCK node" << std::endl;
        return;
    }

    Field *rotField = myEPUCK->getField("rotation");
    if (!rotField)
    {
        std::cerr << "Error: Cannot find rotation field" << std::endl;
        return;
    }

    double targetAngles[4] = {
        M_PI / 2, // NORTH (90 degrees)
        0.0,      // EAST (0 degrees)
        -M_PI / 2,// SOUTH (-90 degrees)
        M_PI      // WEST (180 degrees)
    };

    
    double targetAngle = targetAngles[static_cast<int>(targetHeading)];


    const double kp = 5.0;      
    const double threshold = 0.01; 
    const int maxSteps = 65;  

    for (int stepCount = 0; stepCount < maxSteps; ++stepCount)
    {
        
        const double *rotArray = rotField->getSFRotation();
        double currentAngle = rotArray[3];

        
        double error = targetAngle - currentAngle;

        // Normalize error to range [-π, π]
        while (error > M_PI)
            error -= 2.0 * M_PI;
        while (error < -M_PI)
            error += 2.0 * M_PI;

        
        if (std::fabs(error) < threshold)
        {
            motors.stop();
            std::cout << "Turned to heading: " << static_cast<int>(targetHeading)
                      << ", Final angle: " << currentAngle * 180.0 / M_PI << " degrees" << std::endl;
            heading = targetHeading; // Update internal state
            return;
        }

        
        double turnSpeed = kp * error; // Apply proportional control
        turnSpeed = clamp(turnSpeed, -Config::TURN_SPEED, Config::TURN_SPEED); 

        
        motors.setSpeed(-turnSpeed, turnSpeed);

        
        step(Config::TIME_STEP);
    }

    
    motors.stop();
    std::cerr << "Failed to turn to heading: " << static_cast<int>(targetHeading) << std::endl;
}



void Epuck::run()
{
    std::cout << "E-puck robot starting..." << std::endl;

    turnToHeading(Config::Heading::NORTH);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    // for now starting in north direction
    heading = Config::Heading::NORTH;
    position = recordOwnPosition();

    int startX = position.x_mapped;
    int startY = position.y_mapped;

    floodfill.printMaze();
    floodfill.floodMaze(startX, startY, Config::cellOrder[0].first, Config::cellOrder[0].second);
    floodfill.printCosts();

    sensorManager.readSensors(sensorValues);


    go(*this, sensorValues);

}

char Epuck::floorColor() {
    char color = cameraController.processCamera();
    std::cout << "Detected floor color: " << color << std::endl;
    return color;
}

