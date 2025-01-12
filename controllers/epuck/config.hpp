#ifndef CONFIG_HPP
#define CONFIG_HPP

namespace Config {
    struct Epuck {
        // Time configuration
        static const int TIME_STEP = 64;  // milliseconds

        // Hardware configuration
        static const int NUM_SENSORS = 8;
        static const int NUM_LEDS = 10;
        static constexpr double MAX_SPEED = 6.28;  // rad/s

        // Sensor configuration
        static const int HISTORY_SIZE = 2;
        static const int STABILIZING_THRESHOLD = 8;
        static const int CALIBRATION_SAMPLES = 50;

        // PID configuration
        static constexpr float DEFAULT_KP = 5.0;
        static constexpr float DEFAULT_KD = 1.0;

        // Obstacle avoidance thresholds
        static constexpr double OBSTACLE_THRESHOLD = 0.5;
    };
}

#endif