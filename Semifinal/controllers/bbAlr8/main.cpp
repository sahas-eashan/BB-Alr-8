#include "bbAlr8.hpp"

int main(int argc, char **argv) {
    BbAlr8& robot = BbAlr8::getInstance();
    robot.run();  // This ensures simulation steps occur
    //delete robot;
    return 0;
}