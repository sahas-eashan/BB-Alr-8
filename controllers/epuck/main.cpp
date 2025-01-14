#include "epuck.hpp"

int main(int argc, char **argv) {
    Epuck *robot = new Epuck();
    robot->run();  // This ensures simulation steps occur
    delete robot;
    return 0;
}