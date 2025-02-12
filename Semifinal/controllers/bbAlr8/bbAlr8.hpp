#ifndef BBALR8_HPP
#define BBALR8_HPP

#include <webots/Robot.hpp>
#include <webots/Camera.hpp>
#include "ledManager.hpp"
#include "config.hpp"
#include "sensorManager.hpp"
#include "motors.hpp"
#include "CameraController.hpp"
// #include "floodfill.hpp"

#include "API.hpp"
// #include "solver.hpp"


struct Position {
    int x_mapped, y_mapped;   // Mapped grid coordinates
};

class BbAlr8 : public webots::Robot {
    public:
        static BbAlr8& getInstance() {
            static BbAlr8 instance;
            return instance;
        }
    
        void run();
        char floorColor();
        bool iswallFront();
        bool iswallRight();
        bool iswallLeft();

        void moveForward();
        void turnLeft();
        void turnRight();
        void turn180();
    
    private:
        BbAlr8();  // Private constructor to enforce singleton
        ~BbAlr8(); // Private destructor
        BbAlr8(const BbAlr8&) = delete; // Delete copy constructor
        BbAlr8& operator=(const BbAlr8&) = delete; // Delete assignment operator
    
        void initDevices();
    
        LEDManager leds;
        SensorManager sensorManager;
        Motors motors;
        CameraController cameraController;
    };
    

#endif 