#ifndef BBALR8_HPP
#define BBALR8_HPP

#include <webots/Robot.hpp>
#include <webots/Camera.hpp>
#include "ledManager.hpp"
#include "config.hpp"
#include "sensorManager.hpp"
#include "motors.hpp"
#include "CameraController.hpp"
#include <unordered_map>

// #include "floodfill.hpp"

#include "API.hpp"
// #include "solver.hpp"


struct Position {
    int x_mapped, y_mapped;   // Mapped grid coordinates
};

struct ColorInfo {
    std::string name;
    int8_t value;
};

// Define a color mapping
const std::unordered_map<char, ColorInfo> colorMap = {
    {'R', {"Red", 3}},
    {'O', {"Orange", 2}},
    {'Y', {"Yellow", 1}},
    {'W', {"White", 0}},
    {'U', {"Shadow", 4}}
};

class BbAlr8 : public webots::Robot {
    public:
        static BbAlr8& getInstance() {
            static BbAlr8 instance;
            return instance;
        }
    
        void run();
        int8_t getFloorColor();
        bool iswallFront();
        bool iswallRight();
        bool iswallLeft();

        void move_Forward();
        void turn_Left();
        void turn_Right();
        void turn_180();
    
    private:
        BbAlr8();  // Private constructor to enforce singleton
        ~BbAlr8(); // Private destructor
        BbAlr8(const BbAlr8&) = delete; // Delete copy constructor
        BbAlr8& operator=(const BbAlr8&) = delete; // Delete assignment operator
    
        void initDevices();
        void updateLEDs(int8_t colorValue);
    
        LEDManager leds;
        SensorManager sensorManager;
        Motors motors;
        CameraController cameraController;
    };
    

#endif 