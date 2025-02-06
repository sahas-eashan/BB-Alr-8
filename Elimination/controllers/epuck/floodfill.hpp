#ifndef FLOODFILL_HPP
#define FLOODFILL_HPP

#include "config.hpp"

class Floodfill
{
public:
    Floodfill();
    ~Floodfill();

    bool hasWall(int x, int y, int direction) const;

    // Flooding methods
    void floodMaze(int startX, int startY, int endX, int endY);
    int getCellCost(int x, int y) const;
    void printCosts() const; 
    void printMaze();

private:

    // Wall configuration using bit flags for N,E,S,W walls
    int wallConfig[Config::MAZE_LENGTH][Config::MAZE_WIDTH] = {  // [y][x]
        {13, 4, 6, 12, 6, 14, 14, 12, 4, 6},
        {12, 3, 9, 3, 11, 10, 9, 3, 10, 11},
        {9, 6, 12, 5, 5, 3, 12, 5, 1, 6},
        {14, 10, 9, 6, 12, 7, 11, 12, 5, 3},
        {10, 10, 14, 10, 9, 5, 6, 8, 6, 14},
        {10, 9, 2, 10, 12, 6, 9, 2, 10, 10},
        {10, 12, 2, 9, 3, 9, 5, 3, 8, 3},
        {10, 11, 10, 12, 6, 13, 4, 6, 8, 7},
        {8, 6, 9, 3, 9, 5, 2, 10, 9, 6},
        {11, 9, 5, 5, 5, 5, 3, 9, 5, 3},
    };

    int costsOfCells[Config::MAZE_LENGTH][Config::MAZE_WIDTH] = {};

    // Helper methods
    bool isValidCell(int x, int y) const;
    void updateCellCost(int x, int y, int cost);
    void printMazeCell(int cell, bool isTopRow = false);
    bool canMove(int fromX, int fromY, int toX, int toY) const;

};

#endif