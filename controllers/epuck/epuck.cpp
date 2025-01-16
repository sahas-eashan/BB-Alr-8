#include "epuck.hpp"
#include <iostream>
#include <chrono>
#include <webots/Receiver.hpp>
#include <sstream>
#include <webots/Node.hpp>
#include <webots/Supervisor.hpp>
#include <cmath>  

using namespace webots;

Epuck::Epuck()
{
    initDevices();
}

Epuck::~Epuck()
{
}

void Epuck::initDevices() {
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



Position Epuck::recordOwnPosition() {
    Node *selfNode = getFromDef("EPUCK");  // Ensure the DEF name matches
    if (selfNode == nullptr) {
        std::cerr << "Error: Could not retrieve self node!" << std::endl;
        return { -1, -1};  // Return invalid values in case of an error
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
    }

    motors.stop();
}

void Epuck::faceNorth() {
    // Attempt to grab the EPUCK node from the scene
    Node *myEPUCK = getFromDef("EPUCK");
    if (!myEPUCK) {
        std::cerr << "Uh oh, can't find EPUCK node. No turning possible." << std::endl;
        return;
    }

    // We'll get the 'rotation' field, which is axis-angle: [0,1,0, angle] ideally
    Field *rotField = myEPUCK->getField("rotation");
    if (!rotField) {
        std::cerr << "No rotation field? This is suspicious, dude." << std::endl;
        return;
    }

    // Grab the current rotation array
    const double *rotArray = rotField->getSFRotation();

    // The angle is at index 3
    double currAng = rotArray[3];

    // Let's define "north" as facing 90 degrees (PI/2) about Y-axis
    double northAng = 1.5708; // about 90 deg in radians

    // We'll see how much we need to rotate
    double angleWeNeed = northAng - currAng;

    // Normalize that angle into [-pi, pi] so we turn the shortest way
    while (angleWeNeed > M_PI) angleWeNeed -= 2.0 * M_PI;
    while (angleWeNeed < -M_PI) angleWeNeed += 2.0 * M_PI;

    // Convert to degrees just so we can scale the turning time easily
    double angleDeg = angleWeNeed * 180.0 / M_PI;

    // Our turning speed
    double spinSpeed = Config::TURN_SPEED;

    // Time needed is proportional to how many degrees we must spin
    // we know TIME_90_TURN is the time to turn 90 deg
    double timeToSpin = (std::fabs(angleDeg) / 90.0) * Config::TIME_90_TURN;

    // Decide direction: if angleWeNeed > 0, let's turn left, else turn right
    if (angleWeNeed > 0) {
        // Turn left
        motors.setSpeed(-spinSpeed, spinSpeed);
    } else {
        // Turn right
        motors.setSpeed(spinSpeed, -spinSpeed);
    }

    // Step long enough to complete the turn
    step(static_cast<int>(timeToSpin));
    motors.stop();

    // Just for fun, let's print out the final orientation
    std::cout << "Just tried to face north. Let's hope it worked!" << std::endl;
}

void Epuck::run()
{
    std::cout << "E-puck robot starting..." << std::endl;
    faceNorth();
    Position pos = recordOwnPosition();

    int startX = pos.x_mapped;
    int startY = pos.y_mapped;

    floodfill.floodMaze(startX , startY , Config::cellOrder[0].first, Config::cellOrder[0].second);
    floodfill.printCosts(); 

    // Main control loop
    while (step(Config::TIME_STEP) != -1)
    {
        sensorManager.readSensors(sensorValues);

        moveForward(4, sensorValues);
        turn180();
        
        motors.delay(2000);
        
    }
}