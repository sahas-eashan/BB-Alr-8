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
        //
    }

    // Initialize sensors
    sensorManager.initializeSensors(this);
    std::cout << "Sensors Initialized" << std::endl;
    motors.initializeMotors(this);
    std::cout << "Motors Initialized" << std::endl;
    
     // Grab the PositionSensors from Webots
    leftPosSensor = getPositionSensor("left wheel sensor");
    rightPosSensor = getPositionSensor("right wheel sensor");
    if (!leftPosSensor || !rightPosSensor) {
        std::cerr << "Oops! Could not find wheel sensors. Check your .wbt file for correct names." << std::endl;
    } else {
        leftPosSensor->enable(Config::TIME_STEP);
        rightPosSensor->enable(Config::TIME_STEP);
        std::cout << "PositionSensors enabled." << std::endl;
    }
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


void Epuck::turnLeft() {
    // let's define some robot geometry constants
    double trackWidth   = 0.053;   // distance (meters) between the two e-puck wheels
    double wheelRadius  = 0.0205;  // about 2.05 cm radius for each wheel
    double turnAngleDeg = 90.0;    // we want a 90° turn to the left

    // convert desired angle to radians
    double turnAngleRad = turnAngleDeg * M_PI / 180.0;

    // for an in-place turn, each wheel travels an arc with radius = trackWidth/2
    // arc length = angle_in_radians * radius
    double distancePerWheel = turnAngleRad * (trackWidth / 2.0);

    // how many radians each wheel must rotate
    // (distance = radius_wheel * wheel_rotation)
    double requiredWheelRotation = distancePerWheel / wheelRadius;

    // read the starting angles from the sensors
    double startLeftAngle  = leftPosSensor->getValue();
    double startRightAngle = rightPosSensor->getValue();

    // we’ll set one wheel to go forward, other backward
    double turningSpeed = 3.0; // rad/s (tweak as desired)
    motors.setSpeed(-turningSpeed, turningSpeed); 
    // that should pivot left in place (left wheel backward, right wheel forward)

    // we keep turning until the difference in wheel angles = 2 * requiredWheelRotation
    // Why 2 * ? Because the left wheel is going negative ~X, and the right is going +X.
    bool keepTurning = true;
    while (keepTurning) {
        step(Config::TIME_STEP);

        double leftNow  = leftPosSensor->getValue();
        double rightNow = rightPosSensor->getValue();

        // how much each wheel has rotated from the start
        double leftDelta  = leftNow  - startLeftAngle;
        double rightDelta = rightNow - startRightAngle;

        // For a perfect in-place turn:
        //   leftDelta ~ -requiredWheelRotation
        //   rightDelta ~ +requiredWheelRotation
        // So the difference (rightDelta - leftDelta) should be ~ 2*requiredWheelRotation
        double angleDiff = rightDelta - leftDelta;

        if (angleDiff >= (2.0 * requiredWheelRotation)) {
            keepTurning = false;
        }
    }

    motors.stop();
}

void Epuck::turnRight() {
    double trackWidth   = 0.053;  
    double wheelRadius  = 0.0205; 
    double turnAngleDeg = 90.0;   

    double turnAngleRad = turnAngleDeg * M_PI / 180.0;
    double distancePerWheel = turnAngleRad * (trackWidth / 2.0);
    double requiredWheelRotation = distancePerWheel / wheelRadius;

    double startLeftAngle  = leftPosSensor->getValue();
    double startRightAngle = rightPosSensor->getValue();

    // for a right turn, left wheel forward, right wheel backward
    double turningSpeed = 3.0; // rad/s
    motors.setSpeed(turningSpeed, -turningSpeed);

    bool keepTurning = true;
    while (keepTurning) {
        step(Config::TIME_STEP);

        double leftNow  = leftPosSensor->getValue();
        double rightNow = rightPosSensor->getValue();

        double leftDelta  = leftNow  - startLeftAngle;
        double rightDelta = rightNow - startRightAngle;

        double angleDiff = leftDelta - rightDelta; 
        // because for a right turn, left is positive, right is negative
        // so leftDelta - rightDelta should end up ~ 2*requiredWheelRotation

        if (angleDiff >= (2.0 * requiredWheelRotation)) {
            keepTurning = false;
        }
    }

    motors.stop();
}

void Epuck::turn180() {
    // We want 180 degrees
    double trackWidth   = 0.053; 
    double wheelRadius  = 0.0205;
    double turnAngleDeg = 180.0;

    double turnAngleRad = turnAngleDeg * M_PI / 180.0;
    double distancePerWheel = turnAngleRad * (trackWidth / 2.0);
    double requiredWheelRotation = distancePerWheel / wheelRadius;

    double startLeftAngle  = leftPosSensor->getValue();
    double startRightAngle = rightPosSensor->getValue();

    // For a 180, let's do left wheel forward, right wheel backward, or vice versa
    double turningSpeed = 3.0; 
    motors.setSpeed(turningSpeed, -turningSpeed); // rotate in place

    bool keepTurning = true;
    while (keepTurning) {
        step(Config::TIME_STEP);

        double leftNow  = leftPosSensor->getValue();
        double rightNow = rightPosSensor->getValue();

        double leftDelta  = leftNow  - startLeftAngle;
        double rightDelta = rightNow - startRightAngle;

        // for a 180 right turn, difference is leftDelta - rightDelta ~ 2 * requiredWheelRotation
        double angleDiff = leftDelta - rightDelta;

        if (angleDiff >= (2.0 * requiredWheelRotation)) {
            keepTurning = false;
        }
    }

    motors.stop();
}

void Epuck::moveForward(int cells, double *sensorValues) {
    // Let's define how many meters 1 cell is. 
    // Suppose each cell is 0.1 m wide (example).
    double distPerCell = 0.265;
    double targetDist = cells * distPerCell;

    // e-puck's approximate wheel radius in meters.
    double wheelRadius = 0.0205; // ~2.05 cm

    // read the initial wheel rotation
    double leftStartAngle  = leftPosSensor->getValue();
    double rightStartAngle = rightPosSensor->getValue();

    // We'll loop until we reach the desired average distance
    bool keepMoving = true;

    while (keepMoving) {
        // read sensors for any required steering
        sensorManager.readSensors(sensorValues);
        double correction = sensorManager.calculateSteeringAdjustment();

        // set speeds using base speed +/- the correction
        motors.setSpeed(Config::BASE_SPEED - correction, 
                        Config::BASE_SPEED + correction);

        // do one simulation step
        step(Config::TIME_STEP);

        // get current angles
        double leftNowAngle  = leftPosSensor->getValue();
        double rightNowAngle = rightPosSensor->getValue();

        // convert angle difference to linear distance
        double leftDist  = (leftNowAngle  - leftStartAngle)  * wheelRadius;
        double rightDist = (rightNowAngle - rightStartAngle) * wheelRadius;

        // average distance traveled by both wheels
        double avgDist = (std::fabs(leftDist) + std::fabs(rightDist)) / 2.0;

        // check if we've traveled enough
        if (avgDist >= targetDist) {
            keepMoving = false;
        }
    }

    // stop motors
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

bool Epuck::iswallFront(){
    sensorManager.readSensors(sensorValues);
    float F_Wall_Distance = sensorManager.frontWallDistance();
    std::cout << "Front wall Distance: " << F_Wall_Distance << " mm " << std::endl;
    return (F_Wall_Distance < Config::F_WALL_THRESHOLD) ? true : false;
}

bool Epuck::iswallRight(){
    sensorManager.readSensors(sensorValues);
    float R_Wall_Distance = sensorManager.rightWallDistance();
    std::cout << "Right wall Distance: " << R_Wall_Distance << " mm " << std::endl;
    return (R_Wall_Distance < Config::R_WALL_THRESHOLD) ? true : false;
}

bool Epuck::iswallLeft(){
    sensorManager.readSensors(sensorValues);
    float L_Wall_Distance = sensorManager.leftWallDistance();
    std::cout << "Left wall Distance: " << L_Wall_Distance << " mm " << std::endl;
    return (L_Wall_Distance < Config::L_WALL_THRESHOLD) ? true : false;
}


void Epuck::turnToHeading(Config::Heading targetHeading) {
    // Get the robot's node
    Node *myEPUCK = getFromDef("EPUCK");
    if (!myEPUCK) {
        std::cerr << "Error: Cannot find EPUCK node" << std::endl;
        return;
    }

    // Get the rotation field
    Field *rotField = myEPUCK->getField("rotation");
    if (!rotField) {
        std::cerr << "Error: Cannot find rotation field" << std::endl;
        return;
    }

    // Get current rotation
    const double *rotArray = rotField->getSFRotation();
    double currentAngle = rotArray[3];

    // Convert target heading to radians
    double targetAngle;
    switch(targetHeading) {
        case Config::Heading::NORTH:
            targetAngle = M_PI/2;  // 90 degrees
            break;
        case Config::Heading::EAST:
            targetAngle = 0.0;     // 0 degrees
            break;
        case Config::Heading::SOUTH:
            targetAngle = -M_PI/2; // -90 degrees
            break;
        case Config::Heading::WEST:
            targetAngle = M_PI;    // 180 degrees
            break;
        default:
            std::cerr << "Invalid heading specified" << std::endl;
            return;
    }

    // Calculate required rotation
    double angleToRotate = targetAngle - currentAngle;

    // Normalize angle to [-π, π]
    while (angleToRotate > M_PI) angleToRotate -= 2.0 * M_PI;
    while (angleToRotate < -M_PI) angleToRotate += 2.0 * M_PI;

    // Convert to degrees for timing calculation
    double angleDegrees = angleToRotate * 180.0 / M_PI;

    // Calculate turn time based on angle
    double turnTime = (std::fabs(angleDegrees) / 90.0) * Config::TIME_90_TURN;

    // Execute turn
    if (angleToRotate > 0) {
        // Turn left (counterclockwise)
        motors.setSpeed(-Config::TURN_SPEED, Config::TURN_SPEED);
    } else {
        // Turn right (clockwise)
        motors.setSpeed(Config::TURN_SPEED, -Config::TURN_SPEED);
    }

    // Execute turn for calculated duration
    step(static_cast<int>(turnTime));
    motors.stop();

    // Update internal heading state
    heading = targetHeading;

    // Verify final position
    const double *finalRot = myEPUCK->getField("rotation")->getSFRotation();
    std::cout << "Turned to heading: " << static_cast<int>(targetHeading) 
              << ", Final angle: " << finalRot[3] * 180.0 / M_PI << " degrees" << std::endl;
}



void Epuck::run()
{
    std::cout << "E-puck robot starting..." << std::endl;
    //faceNorth(); // Todo: Implement this function to correctly face north
    turnToHeading(Config::Heading::NORTH);
    // for now starting in north direction
    heading = Config::Heading::NORTH;
    position = recordOwnPosition();

    int startX = position.x_mapped;
    int startY = position.y_mapped;

    floodfill.printMaze();
    //floodfill.floodMaze(startX , startY , Config::cellOrder[0].first, Config::cellOrder[0].second);
    //floodfill.printCosts(); 

    sensorManager.readSensors(sensorValues);

    //turnToHeading(Config::Heading::WEST);

   //API_moveForward(*this, sensorValues);
    // API_turnRight(*this);
    // API_moveForward(*this, sensorValues);
    // API_moveForward(*this, sensorValues);
    // API_turnLeft(*this);

    go(*this, sensorValues);
    
    //bool iswall = API_wallFront(*this);

    //std::cout << "Front Wall: " << iswall << std::endl;

    // Main control loop
    while (step(Config::TIME_STEP) != -1)
    {
        //sensorManager.readSensors(sensorValues);

        //moveForward(2, sensorValues);
        //API_moveForward(*this, sensorValues);

        //turn180();
        //API_moveForward(*this, sensorValues);
       
        

        motors.delay(2000);
        
    }
}

// Get distances from specified sensors
        // double distanceSensor0 = sensorManager.getDistance(0);
        // double distanceSensor7 = sensorManager.getDistance(7);
        // double distanceSensor2 = sensorManager.getDistance(2);
        // double distanceSensor5 = sensorManager.getDistance(5);
        // double FrontWallDistance = sensorManager.frontWallDistance();

// std::cout << "Distance from Sensor 0: " << distanceSensor0 << " mm, " ;
        // std::cout << "Distance from Sensor 7: " << distanceSensor7 << " mm | " ;

        // std::cout << "Distance from Sensor 2: " << distanceSensor2 << " mm, ";
        // std::cout << "Distance from Sensor 5: " << distanceSensor5 << " mm " << std::endl;
        // std::cout << "Front wall Distance: " << FrontWallDistance << " mm " << std::endl;



        // Config::Action nextAction = solver(*this);
    // std::cout << "Next Action: " << nextAction << std::endl;
    
    // API_moveForward(*this, sensorValues);

    // Config::Action nextAction2 = solver(*this);
    // std::cout << "Next Action: " << nextAction2 << std::endl;

    // API_turnRight(*this);

    // Config::Action nextAction3 = solver(*this);
    // std::cout << "Next Action: " << nextAction3 << std::endl;