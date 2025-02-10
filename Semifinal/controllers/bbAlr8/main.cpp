#include "bbAlr8.hpp"

int main(int argc, char **argv) {
    BbAlr8 *robot = new BbAlr8();
    robot->run();  // This ensures simulation steps occur
    delete robot;
    return 0;
}