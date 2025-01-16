#include "API.hpp"
#include <iostream>

bool API_wallFront(Epuck& epuck) {
    return epuck.iswallFront();
}

bool API_wallRight(Epuck& epuck) {
    return epuck.iswallRight();
}

bool API_wallLeft(Epuck& epuck) {
    return epuck.iswallLeft();
}

void API_moveForward(Epuck& epuck, double* sensorValues) {
    epuck.moveForward(1, sensorValues);
}

void API_turnRight(Epuck& epuck) {
    epuck.turnRight();
}

void API_turnLeft(Epuck& epuck) {
    epuck.turnLeft();
}

void go(Epuck& epuck, double* sensorValues){
    while (true)
    {
        Config::Action nextMove = solver(epuck);
        //std::cout << "Next Move: " << nextMove << std::endl;

        switch (nextMove)
        {
        case Config::Action::FORWARD:
            API_moveForward(epuck, sensorValues);
            break;
        case Config::Action::RIGHT:
            API_turnRight(epuck);
            break;
        case Config::Action::LEFT:
            API_turnLeft(epuck);
            break;
        case Config::Action::IDLE:
            break;
        
        default:
            break;
        }

        if(nextMove == Config::Action::IDLE){
            break;
        }

    }
    
}