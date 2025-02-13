#include "API.hpp"
#include <iostream>
#include <string>
#include <stdexcept>
#include <thread>
#include <chrono>



bool API_wallFront() {
    return BbAlr8::getInstance().iswallFront();
}

bool API_wallRight() {
    return BbAlr8::getInstance().iswallRight();
}

bool API_wallLeft() {
    return BbAlr8::getInstance().iswallLeft();
}

void API_moveForward() {
    BbAlr8::getInstance().move_Forward();
}

void API_turnLeft() {
    BbAlr8::getInstance().turn_Left();
}

void API_turnRight() {
    BbAlr8::getInstance().turn_Right();
}

void API_turn180() {
    BbAlr8::getInstance().turn_180();
}


void exploreMaze() {
    std::cout << "Starting to explore the maze....." << std::endl;

    MazeSolver explorer;

    //API_moveForward();
    //API_turnRight();
    //API_turn180();  

    //explorer.setStart({10, 0});
    explorer.setTarget({10, 0});

    int i =0;
    while (true) {
        i++;
        std::cout << i << " th move";
        Action action = explorer.explore();
        //std::cout <<" Next Move: " << action << std::endl;
        if (action == Action::ALLEXPLORED) {
            std::cout << "All cells explored!" << std::endl;
            break;
        }
        switch (action) {
            case Action::FORWARD:
                std::cout <<" Next Move: F"<< std::endl;
                API_moveForward();
                break;
            case Action::LEFT:
                std::cout <<" Next Move: L"<< std::endl;
                API_turnLeft();
                break;
            case Action::RIGHT:
                std::cout <<" Next Move: R"<< std::endl;
                API_turnRight();
                break;
            case Action::IDLE:
                break;
            case Action::ALLEXPLORED:
                break;
            default:
                break;  
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    std::cout << "Exploration complete!" << std::endl;
    std::cout << "Returning to start....." << std::endl;

    while (true) {
        Action action = explorer.solve();
        switch (action) {
            case Action::FORWARD:
                API_moveForward();
                break;
            case Action::LEFT:
                API_turnLeft();
                break;
            case Action::RIGHT:
                API_turnRight();
                break;
            case Action::IDLE:
                break;
            case Action::ALLEXPLORED:
                break;
            default:
                break; 
        }
    }


}