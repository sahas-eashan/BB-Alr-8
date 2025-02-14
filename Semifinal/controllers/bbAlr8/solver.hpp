#ifndef SOLVER_H
#define SOLVER_H

#include <array>
#include <vector>
#include <stack>


enum class Heading { NORTH, EAST, SOUTH, WEST };
enum class Action { LEFT, FORWARD, RIGHT, IDLE, ALLEXPLORED };

// Maze constants
constexpr int MAZE_SIZE = 20;

// Wall configurations in binary format: TopWall RightWall BottomWall LeftWall
namespace WallConfig {
    constexpr unsigned int NONE  = 0b0000;
    constexpr unsigned int W     = 0b0001;
    constexpr unsigned int S     = 0b0010;
    constexpr unsigned int SW    = 0b0011;
    constexpr unsigned int E     = 0b0100;
    constexpr unsigned int EW    = 0b0101;
    constexpr unsigned int ES    = 0b0110;
    constexpr unsigned int ESW   = 0b0111;
    constexpr unsigned int N     = 0b1000;
    constexpr unsigned int NW    = 0b1001;
    constexpr unsigned int NS    = 0b1010;
    constexpr unsigned int NSW   = 0b1011;
    constexpr unsigned int NE    = 0b1100;
    constexpr unsigned int NEW   = 0b1101;
    constexpr unsigned int NES   = 0b1110;
    constexpr unsigned int NESW  = 0b1111;  // not actually possible in a maze
}

struct Coordinate {
    int x;
    int y;
};

class MazeSolver {
public:
    MazeSolver();
    Action solve();
    //Action leftWallFollower();
    Action tremauxSearch();
    Action explore();
    //std::stack<int> path;

    // Add getter for distances array
    const std::array<std::array<int, MAZE_SIZE>, MAZE_SIZE>& getDistances() const { 
        return distances; 
    }

    void setTarget(Coordinate target) {
        targetCoordinate = target;
        resetDistances(target);
    }

    void setStart(Coordinate start) {
        position = start;
        resetDistances(targetCoordinate);
    }

    void printMaze() const;

    Action dfsSearch(); 


private:
    std::array<std::array<unsigned int, MAZE_SIZE>, MAZE_SIZE> maze;
    std::array<std::array<int, MAZE_SIZE>, MAZE_SIZE> distances;
    Coordinate position;
    Coordinate targetCoordinate;
    Heading heading;
    bool reachedCenter;

    bool exploredAll;

    void initialize();
    void updateMaze();
    void updateDistances();
    void resetDistances(Coordinate target);
    int xyToSquare(int x, int y) const;
    Coordinate squareToCoord(int square) const;
    bool isWallInDirection(int x, int y, Heading direction) const;
    void updateHeading(Action nextAction);
    void updatePosition(Action nextAction);
    Action floodFill() const;

    std::array<std::array<int, MAZE_SIZE>, MAZE_SIZE> visitCount{};  
    bool explorationMode{true}; 
    bool allCellsExplored();

    void updateColour();
    void addDangerZone(int x, int y);

    std::stack<Coordinate> dfsStack;
    bool backtracking;
    Coordinate previousPosition;

    bool shouldMarkVisited(int x, int y);
};

#endif