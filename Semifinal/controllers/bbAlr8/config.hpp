#ifndef CONFIG_HPP
#define CONFIG_HPP

#include "math.h"


namespace Config
{
    // Time configuration
    static const int TIME_STEP = 16; // milliseconds

    // Hardware configuration
    static const int NUM_SENSORS = 6;
    static const int NUM_LEDS = 4;

    // Sensors
    static const int STABILIZING_THRESHOLD = 10;
    static const int SENSOR_MAX = 4095;
    static const int SENSOR_MIN = 0;

    static const int MAX_DISTANCE = 35; // cm
    static const int MIN_DISTANCE = 0;

    static const double IDEAL_WALL_DISTANCE = 9.3; // Desired distance from wall in cm
    static const int MAX_WALL_DISTANCE = 30;
    static const double MAX_STEERING = 1.6; // Maximum steering adjustment
    static const double MAX_INTEGRAL = 1.0; // Anti-windup limit
    static const int ALIGN_DISTANCE = 10;

    static const double Kp = 0.7;
    static const double Ki = 0.01;
    static const double Kd = 0.11;

    // Motors
    const int TIME_90_TURN = 350; 
    const int TIME_180_TURN = 680;
    const int TIME_PER_CELL = 510; 


    static constexpr double MAX_SPEED = 40.0; 
    constexpr double BASE_SPEED = Config::MAX_SPEED * 0.7;
    constexpr double TURN_SPEED = Config::MAX_SPEED * 0.2; 

    // Floodfill
    const int MAZE_LENGTH = 20;
    const int MAZE_WIDTH = 20;

    typedef enum Heading
    {
        NORTH,
        EAST,
        SOUTH,
        WEST
    } Heading;

    typedef enum Action
    {
        LEFT,
        FORWARD,
        RIGHT,
        IDLE,
        NONE
    } Action;

    // Threshold values for detecting walls
    const float L_WALL_THRESHOLD = 23;
    const float F_WALL_THRESHOLD = 16;
    const float R_WALL_THRESHOLD = 23;

    const int GREEN_PIXEL_COUNT = 19;

    const int DELAY_TIME  = 3000;

}
#endif