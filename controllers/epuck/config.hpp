#ifndef CONFIG_HPP
#define CONFIG_HPP

#include "math.h"


namespace Config
{
    // Time configuration
    static const int TIME_STEP = 16; // milliseconds

    // Hardware configuration
    static const int NUM_SENSORS = 8;
    static const int NUM_LEDS = 10;

    // Sensors
    static const int STABILIZING_THRESHOLD = 50;
    static const int SENSOR_MAX = 4095;
    static const int SENSOR_MIN = 0;

    static const int MAX_DISTANCE = 35; // cm
    static const int MIN_DISTANCE = 0;

    static const double IDEAL_WALL_DISTANCE = 9; // Desired distance from wall in cm
    static const int MAX_WALL_DISTANCE = 25;
    static const double MAX_STEERING = 2.0; // Maximum steering adjustment
    static const double MAX_INTEGRAL = 1.0; // Anti-windup limit

    static const double Kp = 0.8;
    static const double Ki = 0.1;
    static const double Kd = 0.2;

    // Motors
    constexpr double WHEEL_RADIUS = 0.0205; 
    constexpr double WHEEL_DISTANCE = 0.0568;                           
    constexpr double WHEEL_CIRCUMFERENCE = 2 * M_PI * WHEEL_RADIUS;     
    constexpr double MAX_SPEED = 10.0;                                  
    constexpr double MAX_SPEED_MPS = MAX_SPEED * WHEEL_RADIUS;     

    constexpr double CELL_SIZE = 0.180;          // [m] Size of one maze cell
    constexpr double TURNING_SPEED = 2.5;        // [rad/s] Speed during turns
    constexpr double POSITION_TOLERANCE = 0.001; // [m] Acceptable position error
    constexpr double ANGLE_TOLERANCE = 0.01;     // [rad] Acceptable angle error

    // Movement timing constants
    constexpr int TURN_90_STEPS = 375;   // Time steps for 90-degree turn
    constexpr int TURN_180_STEPS = 750;  // Time steps for 180-degree turn
    constexpr double ACCELERATION = 0.3; // [rad/s²] Acceleration rate
    constexpr double DECELERATION = 0.3; // [rad/s²] Deceleration rate

    // Encoder-related constants
    constexpr double ENCODER_RESOLUTION = 159.23; // [steps/rotation] E-puck encoder resolution
    constexpr double STEPS_PER_METER = ENCODER_RESOLUTION / WHEEL_CIRCUMFERENCE;

    // Movement thresholds
    constexpr double MIN_SPEED = 0.1;                                            // [rad/s] Minimum speed to consider robot moving
    constexpr double ROTATION_COEFFICIENT = WHEEL_DISTANCE / (2 * WHEEL_RADIUS); // For turn calculations
}
#endif