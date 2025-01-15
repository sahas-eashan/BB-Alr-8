#ifndef FLOODFILL_HPP
#define FLOODFILL_HPP

#include "config.hpp"

class Floodfill {
public:
    Floodfill();
    ~Floodfill();

    // Wall configuration methods
    void setWall(int x, int y, int direction);
    bool hasWall(int x, int y, int direction) const;
    
    // Flooding methods
    void floodMaze(int startX, int startY, int endX, int endY);
    int getCellCost(int x, int y) const;

private:
    // Wall configuration using bit flags for N,E,S,W walls
    int wallConfig[Config::MAZE_LENGTH][Config::MAZE_WIDTH] = {};
    int costsOfCells[Config::MAZE_LENGTH][Config::MAZE_WIDTH];

    // Helper methods
    bool isValidCell(int x, int y) const;
    void updateCellCost(int x, int y, int cost);
};

#endif