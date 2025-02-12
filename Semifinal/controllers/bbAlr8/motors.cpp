#include "motors.hpp"
#include <thread>

Motors::Motors() {}

Motors::~Motors() {}

void Motors::initializeMotors(webots::Robot *robot)
{
    leftMotor = robot->getMotor("leftMotor");
    rightMotor = robot->getMotor("rightMotor");

    leftMotor->setPosition(INFINITY);
    rightMotor->setPosition(INFINITY);

    setSpeed(0.0, 0.0);
}

void Motors::setSpeed(double leftSpeed, double rightSpeed)
{

    leftSpeed = clamp(leftSpeed, -Config::MAX_SPEED, Config::MAX_SPEED);
    rightSpeed = clamp(rightSpeed, -Config::MAX_SPEED, Config::MAX_SPEED);

    leftMotor->setVelocity(-leftSpeed);
    rightMotor->setVelocity(-rightSpeed); // negative because motor direction is inverted
}

void Motors::stop()
{
    setSpeed(0.0, 0.0);
}

void Motors::turnLeft(webots::Robot *robot)
{
    setSpeed(Config::TURN_SPEED, -Config::TURN_SPEED);
    robot->step(Config::TIME_90_TURN);
    stop();
}

void Motors::turnRight(webots::Robot *robot)
{
    setSpeed(-Config::TURN_SPEED, Config::TURN_SPEED);
    robot->step(Config::TIME_90_TURN);
    stop();
}

void Motors::turn180(webots::Robot *robot)
{
    setSpeed(-Config::TURN_SPEED, +Config::TURN_SPEED);
    robot->step(Config::TIME_180_TURN);
    stop();
}

void Motors::moveForward(webots::Robot *robot, SensorManager sensorManager, int cells)
{
    int totalTime = cells * Config::TIME_PER_CELL;
    auto startTime = std::chrono::steady_clock::now();

    while (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - startTime).count() < totalTime)
    {
        sensorManager.readSensors();

        if (sensorManager.frontWallDistance() < Config::ALIGN_DISTANCE)
        {
            break;
        }

        double correction = sensorManager.calculateSteeringAdjustment();
        setSpeed(Config::BASE_SPEED + correction, Config::BASE_SPEED - correction);

        robot->step(Config::TIME_STEP);
    }

    if (sensorManager.iswallFront())
    {
        while (sensorManager.frontWallDistance() > Config::ALIGN_DISTANCE)
        {
            sensorManager.readSensors();
            double correction = sensorManager.calculateSteeringAdjustment();
            setSpeed(Config::BASE_SPEED + correction, Config::BASE_SPEED - correction);

            robot->step(Config::TIME_STEP);
        }
    }

    stop();

    // if (sensorManager.iswallFront())
    // {
    //     // Align the robot with the wall
    //     while (true)
    //     {
    //         sensorManager.readSensors();
    //         double rightDistance = sensorManager.frontRightDistance();
    //         double leftDistance = sensorManager.frontLeftDistance();
    //         double error2 = rightDistance - leftDistance;
    //         std::cout << "Error: " << error2 << std::endl;

    //         if (abs(error2) < 0.05 && sensorManager.frontWallDistance() < Config::ALIGN_DISTANCE) // Adjust the threshold as needed
    //             break;

    //         double correction2 = error2 * 10; // adjust as necessary
    //         double leftSpeed = correction2;
    //         double rightSpeed = -correction2;

    //         setSpeed(leftSpeed, rightSpeed);
    //         robot->step(Config::TIME_STEP);
    //     }
    // }

    // stop();
}

void Motors::enterMaze(webots::Robot *robot, SensorManager sensorManager)
{
    setSpeed(Config::BASE_SPEED, Config::BASE_SPEED);

    while (true)
    {
        sensorManager.readSensors();

        // Check if there's a wall in front
        if (sensorManager.iswallFront())
        {
            stop();

            // Align the robot with the wall
            while (true)
            {
                sensorManager.readSensors();
                double rightDistance = sensorManager.frontRightDistance();
                double leftDistance = sensorManager.frontLeftDistance();
                double error = rightDistance - leftDistance;
                std::cout << "Error: " << error << std::endl;

                if (abs(error) < 0.02 && sensorManager.frontWallDistance() < Config::ALIGN_DISTANCE) // Adjust the threshold as needed
                    break;

                double correction = error * 0.3; // 0.5 is the gain, adjust as necessary
                double leftSpeed = Config::BASE_SPEED + correction;
                double rightSpeed = Config::BASE_SPEED - correction;

                setSpeed(leftSpeed, rightSpeed);
                robot->step(Config::TIME_STEP);
            }

            stop();
            break;
        }

        // Continue moving forward
        robot->step(Config::TIME_STEP);
    }
}

void Motors::delay(int ms)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}