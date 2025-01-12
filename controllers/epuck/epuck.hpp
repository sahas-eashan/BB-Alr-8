#ifndef EPUCK_HPP
#define EPUCK_HPP

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/LED.hpp>

class Epuck : public webots::Robot
{
public:
    Epuck();
    virtual ~Epuck();
    void run();

private:
    // Time step in milliseconds
    static const int TIME_STEP = 64;

    // Sensors variables
    static const int NUM_SENSORS = 8;
    static const int NUM_LEDS = 10;
    static constexpr double MAX_SPEED = 6.28; // rad/s
    static const int STABILIZING_THRESHOLD =8;

    bool pidEnabled = true;
    float lastError = 0.0;

    float Kp = 5.0;
    float Kd = 1.0;

    static const int HISTORY_SIZE = 2; // Size of moving average window
    double sensorHistory[NUM_SENSORS][HISTORY_SIZE] = {0};
    int historyIndex = 0;
    double baselineValues[NUM_SENSORS] ={0}; // Stores ambient values
    bool isBaselineInitialized = false;
    webots::DistanceSensor *distanceSensors[NUM_SENSORS];

    // Motors variables and pointers
    webots::Motor *leftMotor;
    webots::Motor *rightMotor;

    webots::LED *leds[NUM_LEDS];

    // Helper functions
    void initDevices();
    void setMotorSpeeds(double leftSpeed, double rightSpeed);

    // Sensors
    void initSensors();
    void calibrateSensors();
    void readSensors(double *sensorValues);
    void togglePID();
    float calculatePID(float leftValue, float rightValue);
};

#endif