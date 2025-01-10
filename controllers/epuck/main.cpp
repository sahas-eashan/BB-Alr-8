#include "epuck.hpp"

int main(int argc, char **argv) {
    Epuck* robot = new Epuck();
    robot->run();
    delete robot;
    return 0;
}