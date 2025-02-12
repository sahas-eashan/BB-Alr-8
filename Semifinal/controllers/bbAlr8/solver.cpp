#include "solver.hpp"
#include "API.hpp"
#include <limits>
#include <queue>
#include <algorithm>

MazeSolver::MazeSolver() 
    : maze{}, distances{}, position{0, 0}, heading{Heading::NORTH}, reachedCenter{false} {
    initialize();
}

void MazeSolver::initialize() {
    // Setting the borders
    for (int i = 1; i < MAZE_SIZE - 1; ++i) {
        maze[0][i] = WallConfig::W;
        maze[i][0] = WallConfig::S;
        maze[i][MAZE_SIZE - 1] = WallConfig::N;
        maze[MAZE_SIZE - 1][i] = WallConfig::E;
    }
    maze[0][0] = WallConfig::SW;
    maze[0][MAZE_SIZE - 1] = WallConfig::NW;
    maze[MAZE_SIZE - 1][0] = WallConfig::ES;
    maze[MAZE_SIZE - 1][MAZE_SIZE - 1] = WallConfig::NE;

    resetDistances();

}

void MazeSolver::updateMaze() {
    int x = position.x;
    int y = position.y;
    unsigned int walls = WallConfig::NONE;

    switch (heading) {
    case Heading::NORTH:
        if (API::wallFront()) {
            walls |= WallConfig::N;
            if (y + 1 != MAZE_SIZE)
                maze[x][y + 1] |= WallConfig::S;
        }
        if (API::wallLeft()) {
            walls |= WallConfig::W;
            if (x - 1 >= 0)
                maze[x - 1][y] |= WallConfig::E;
        }
        if (API::wallRight()) {
            walls |= WallConfig::E;
            if (x + 1 != MAZE_SIZE)
                maze[x + 1][y] |= WallConfig::W;
        }
        break;
    case Heading::EAST:
        if (API::wallFront()) {
            walls |= WallConfig::E;
            if (x + 1 != MAZE_SIZE)
                maze[x + 1][y] |= WallConfig::W;
        }
        if (API::wallLeft()) {
            walls |= WallConfig::N;
            if (y + 1 != MAZE_SIZE)
                maze[x][y + 1] |= WallConfig::S;
        }
        if (API::wallRight()) {
            walls |= WallConfig::S;
            if (y - 1 >= 0)
                maze[x][y - 1] |= WallConfig::N;
        }
        break;
    case Heading::SOUTH:
        if (API::wallFront()) {
            walls |= WallConfig::S;
            if (y - 1 >= 0)
                maze[x][y - 1] |= WallConfig::N;
        }
        if (API::wallLeft()) {
            walls |= WallConfig::E;
            if (x + 1 != MAZE_SIZE)
                maze[x + 1][y] |= WallConfig::W;
        }
        if (API::wallRight()) {
            walls |= WallConfig::W;
            if (x - 1 >= 0)
                maze[x - 1][y] |= WallConfig::E;
        }
        break;
    case Heading::WEST:
        if (API::wallFront()) {
            walls |= WallConfig::W;
            if (x - 1 >= 0)
                maze[x - 1][y] |= WallConfig::E;
        }
        if (API::wallLeft()) {
            walls |= WallConfig::S;
            if (y - 1 >= 0)
                maze[x][y - 1] |= WallConfig::N;
        }
        if (API::wallRight()) {
            walls |= WallConfig::N;
            if (y + 1 != MAZE_SIZE)
                maze[x][y + 1] |= WallConfig::S;
        }
        break;
    }

    maze[x][y] |= walls;

    // Update walls in UI
    if (maze[x][y] & WallConfig::N) API::setWall(x, y, 'n');
    if (maze[x][y] & WallConfig::E) API::setWall(x, y, 'e');
    if (maze[x][y] & WallConfig::S) API::setWall(x, y, 's');
    if (maze[x][y] & WallConfig::W) API::setWall(x, y, 'w');
}

int MazeSolver::xyToSquare(int x, int y) const {
    return x + MAZE_SIZE * y;
}

Coordinate MazeSolver::squareToCoord(int square) const {
    return {square % MAZE_SIZE, square / MAZE_SIZE};
}

void MazeSolver::resetDistances() {
    // Initially set all distances to -1 (invalid distance)
    for (auto& row : distances) {
        row.fill(-1);
    }

    // Set goal distances based on whether center has been reached
    if (!reachedCenter) {
        if (MAZE_SIZE % 2 == 0) {
            distances[MAZE_SIZE/2][MAZE_SIZE/2] = 0;
            distances[MAZE_SIZE/2 - 1][MAZE_SIZE/2] = 0;
            distances[MAZE_SIZE/2][MAZE_SIZE/2 - 1] = 0;
            distances[MAZE_SIZE/2 - 1][MAZE_SIZE/2 - 1] = 0;
        } else {
            distances[MAZE_SIZE/2][MAZE_SIZE/2] = 0;
        }
    } else {
        distances[0][0] = 0;  // Go back to start
    }
}

bool MazeSolver::isWallInDirection(int x, int y, Heading direction) const {
    switch (direction) {
    case Heading::NORTH: return maze[x][y] & WallConfig::N;
    case Heading::EAST:  return maze[x][y] & WallConfig::E;
    case Heading::SOUTH: return maze[x][y] & WallConfig::S;
    case Heading::WEST:  return maze[x][y] & WallConfig::W;
    }
    return false;
}

void MazeSolver::updateDistances() {
    resetDistances();
    std::queue<int> squares;

    // Add goal squares to queue
    for (int x = 0; x < MAZE_SIZE; ++x) {
        for (int y = 0; y < MAZE_SIZE; ++y) {
            if (distances[x][y] == 0) {
                squares.push(xyToSquare(x, y));
            }
        }
    }

    while (!squares.empty()) {
        Coordinate square = squareToCoord(squares.front());
        squares.pop();
        int x = square.x;
        int y = square.y;

        // Check all four directions
        const std::array<std::pair<Heading, std::pair<int, int>>, 4> directions = {{
            {Heading::NORTH, {0, 1}},
            {Heading::EAST,  {1, 0}},
            {Heading::SOUTH, {0, -1}},
            {Heading::WEST,  {-1, 0}}
        }};

        for (const auto& [dir, offset] : directions) {
            int newX = x + offset.first;
            int newY = y + offset.second;
            
            if (!isWallInDirection(x, y, dir) && 
                newX >= 0 && newX < MAZE_SIZE && 
                newY >= 0 && newY < MAZE_SIZE && 
                distances[newX][newY] == -1) {
                    distances[newX][newY] = distances[x][y] + 1;
                    squares.push(xyToSquare(newX, newY));
            }
        }
    }
}

void MazeSolver::updateHeading(Action nextAction) {
    if (nextAction == Action::FORWARD || nextAction == Action::IDLE) return;

    const std::array<Heading, 4> headings = {
        Heading::NORTH, Heading::EAST, Heading::SOUTH, Heading::WEST
    };

    int currentIndex = static_cast<int>(heading);
    if (nextAction == Action::LEFT) {
        heading = headings[(currentIndex + 3) % 4];  // +3 is same as -1 but avoids negative numbers
    } else if (nextAction == Action::RIGHT) {
        heading = headings[(currentIndex + 1) % 4];
    }
}

void MazeSolver::updatePosition(Action nextAction) {
    if (nextAction != Action::FORWARD) return;

    const std::array<std::pair<int, int>, 4> offsets = {{
        {0, 1},   // NORTH
        {1, 0},   // EAST
        {0, -1},  // SOUTH
        {-1, 0}   // WEST
    }};

    int index = static_cast<int>(heading);
    position.x += offsets[index].first;
    position.y += offsets[index].second;
}

Action MazeSolver::solve() {
    // Check if center reached or returned to start
    if (!reachedCenter && distances[position.x][position.y] == 0) {
        reachedCenter = true;
    } else if (reachedCenter && distances[position.x][position.y] == 0) {
        reachedCenter = false;
    }

    //Action testaction = tremauxSearch();
    updateMaze();
    updateDistances();

    Action action = floodFill();
    updateHeading(action);
    updatePosition(action);
    return action;
}

bool MazeSolver::allCellsExplored(){
    for (int x = 0; x < MAZE_SIZE; ++x) {
        for (int y = 0; y < MAZE_SIZE; ++y) {
            if (visitCount[x][y] == 0) {
                return false;
            }
        }
    }
    return true;
}

Action MazeSolver::explore(){

    
    if(allCellsExplored()){
        API::debugLog("All cells explored!");
        return Action::IDLE;
    }

    visitCount[position.x][position.y]++;
    

    char color = visitCount[position.x][position.y] == 1 ? 'G' : visitCount[position.x][position.y] == 2 ? 'O' : 'R';  
    API::setColor(position.x, position.y, color);

    updateMaze();
    Action action = tremauxSearch();
    updateHeading(action);
    updatePosition(action);
    return action;
    

    
}

Action MazeSolver::tremauxSearch(){
    unsigned int leastVisits = std::numeric_limits<unsigned int>::max();
    Action optimalMove = Action::IDLE;

    // visitCount[position.x][position.y]++;
    
    //char color = visitCount[position.x][position.y] == 1 ? 'G' : visitCount[position.x][position.y] == 2 ? 'O' : 'R';  
    //API::setColor(position.x, position.y, color);

    if (explorationMode && allCellsExplored()) {
        explorationMode = false;
        API::debugLog("Switching to return mode - all cells visited");
    }

    if (heading == Heading::NORTH){
        // Check forward (North)
        if (!isWallInDirection(position.x, position.y, Heading::NORTH) && 
            position.y + 1 < MAZE_SIZE && 
            visitCount[position.x][position.y + 1] < leastVisits) {
            leastVisits = visitCount[position.x][position.y + 1];
            optimalMove = Action::FORWARD;
            API::debugLog("Debug Point" + std::to_string(leastVisits));
            
        }
        // Check right (East)
        if (!isWallInDirection(position.x, position.y, Heading::EAST) && 
            position.x + 1 < MAZE_SIZE && 
            visitCount[position.x + 1][position.y] < leastVisits) {
            leastVisits = visitCount[position.x + 1][position.y];
            optimalMove = Action::RIGHT;
        }
        // Check left (West)
        if (!isWallInDirection(position.x, position.y, Heading::WEST) && 
            position.x - 1 >= 0 && 
            visitCount[position.x - 1][position.y] < leastVisits) {
            leastVisits = visitCount[position.x - 1][position.y];
            optimalMove = Action::LEFT;
        }
    }
    else if (heading == Heading::EAST) {
        // Check forward (East)
        if (!isWallInDirection(position.x, position.y, Heading::EAST) && 
            position.x + 1 < MAZE_SIZE && 
            visitCount[position.x + 1][position.y] < leastVisits) {
            leastVisits = visitCount[position.x + 1][position.y];
            optimalMove = Action::FORWARD;
        }
        // Check right (South)
        if (!isWallInDirection(position.x, position.y, Heading::SOUTH) && 
            position.y - 1 >= 0 && 
            visitCount[position.x][position.y - 1] < leastVisits) {
            leastVisits = visitCount[position.x][position.y - 1];
            optimalMove = Action::RIGHT;
        }
        // Check left (North)
        if (!isWallInDirection(position.x, position.y, Heading::NORTH) && 
            position.y + 1 < MAZE_SIZE && 
            visitCount[position.x][position.y + 1] < leastVisits) {
            leastVisits = visitCount[position.x][position.y + 1];
            optimalMove = Action::LEFT;
        }
    }
    else if (heading == Heading::SOUTH) {
        // Check forward (South)
        if (!isWallInDirection(position.x, position.y, Heading::SOUTH) && 
            position.y - 1 >= 0 && 
            visitCount[position.x][position.y - 1] < leastVisits) {
            leastVisits = visitCount[position.x][position.y - 1];
            optimalMove = Action::FORWARD;
        }
        // Check right (West)
        if (!isWallInDirection(position.x, position.y, Heading::WEST) && 
            position.x - 1 >= 0 && 
            visitCount[position.x - 1][position.y] < leastVisits) {
            leastVisits = visitCount[position.x - 1][position.y];
            optimalMove = Action::RIGHT;
        }
        // Check left (East)
        if (!isWallInDirection(position.x, position.y, Heading::EAST) && 
            position.x + 1 < MAZE_SIZE && 
            visitCount[position.x + 1][position.y] < leastVisits) {
            leastVisits = visitCount[position.x + 1][position.y];
            optimalMove = Action::LEFT;
        }
    }
    else if (heading == Heading::WEST) {
        // Check forward (West)
        if (!isWallInDirection(position.x, position.y, Heading::WEST) && 
            position.x - 1 >= 0 && 
            visitCount[position.x - 1][position.y] < leastVisits) {
            leastVisits = visitCount[position.x - 1][position.y];
            optimalMove = Action::FORWARD;
        }
        // Check right (North)
        if (!isWallInDirection(position.x, position.y, Heading::NORTH) && 
            position.y + 1 < MAZE_SIZE && 
            visitCount[position.x][position.y + 1] < leastVisits) {
            leastVisits = visitCount[position.x][position.y + 1];
            optimalMove = Action::RIGHT;
        }
        // Check left (South)
        if (!isWallInDirection(position.x, position.y, Heading::SOUTH) && 
            position.y - 1 >= 0 && 
            visitCount[position.x][position.y - 1] < leastVisits) {
            leastVisits = visitCount[position.x][position.y - 1];
            optimalMove = Action::LEFT;
        }
    }
    // Handle dead ends
    if (leastVisits == std::numeric_limits<unsigned int>::max()) {
        optimalMove = Action::RIGHT;
    }
    if(optimalMove == Action::FORWARD){
        visitCount[position.x][position.y]++;
    }

    return optimalMove;
}

Action MazeSolver::floodFill() const {
    unsigned int leastDistance = std::numeric_limits<unsigned int>::max();
    Action optimalMove = Action::IDLE;

    // Check relative positions based on current heading
    if (heading == Heading::NORTH) {
        // Check forward (North)
        if (!isWallInDirection(position.x, position.y, Heading::NORTH) && 
            position.y + 1 < MAZE_SIZE && 
            distances[position.x][position.y + 1] < leastDistance) {
            leastDistance = distances[position.x][position.y + 1];
            optimalMove = Action::FORWARD;
        }
        // Check right (East)
        if (!isWallInDirection(position.x, position.y, Heading::EAST) && 
            position.x + 1 < MAZE_SIZE && 
            distances[position.x + 1][position.y] < leastDistance) {
            leastDistance = distances[position.x + 1][position.y];
            optimalMove = Action::RIGHT;
        }
        // Check left (West)
        if (!isWallInDirection(position.x, position.y, Heading::WEST) && 
            position.x - 1 >= 0 && 
            distances[position.x - 1][position.y] < leastDistance) {
            leastDistance = distances[position.x - 1][position.y];
            optimalMove = Action::LEFT;
        }
    }
    else if (heading == Heading::EAST) {
        // Check forward (East)
        if (!isWallInDirection(position.x, position.y, Heading::EAST) && 
            position.x + 1 < MAZE_SIZE && 
            distances[position.x + 1][position.y] < leastDistance) {
            leastDistance = distances[position.x + 1][position.y];
            optimalMove = Action::FORWARD;
        }
        // Check right (South)
        if (!isWallInDirection(position.x, position.y, Heading::SOUTH) && 
            position.y - 1 >= 0 && 
            distances[position.x][position.y - 1] < leastDistance) {
            leastDistance = distances[position.x][position.y - 1];
            optimalMove = Action::RIGHT;
        }
        // Check left (North)
        if (!isWallInDirection(position.x, position.y, Heading::NORTH) && 
            position.y + 1 < MAZE_SIZE && 
            distances[position.x][position.y + 1] < leastDistance) {
            leastDistance = distances[position.x][position.y + 1];
            optimalMove = Action::LEFT;
        }
    }
    else if (heading == Heading::SOUTH) {
        // Check forward (South)
        if (!isWallInDirection(position.x, position.y, Heading::SOUTH) && 
            position.y - 1 >= 0 && 
            distances[position.x][position.y - 1] < leastDistance) {
            leastDistance = distances[position.x][position.y - 1];
            optimalMove = Action::FORWARD;
        }
        // Check right (West)
        if (!isWallInDirection(position.x, position.y, Heading::WEST) && 
            position.x - 1 >= 0 && 
            distances[position.x - 1][position.y] < leastDistance) {
            leastDistance = distances[position.x - 1][position.y];
            optimalMove = Action::RIGHT;
        }
        // Check left (East)
        if (!isWallInDirection(position.x, position.y, Heading::EAST) && 
            position.x + 1 < MAZE_SIZE && 
            distances[position.x + 1][position.y] < leastDistance) {
            leastDistance = distances[position.x + 1][position.y];
            optimalMove = Action::LEFT;
        }
    }
    else if (heading == Heading::WEST) {
        // Check forward (West)
        if (!isWallInDirection(position.x, position.y, Heading::WEST) && 
            position.x - 1 >= 0 && 
            distances[position.x - 1][position.y] < leastDistance) {
            leastDistance = distances[position.x - 1][position.y];
            optimalMove = Action::FORWARD;
        }
        // Check right (North)
        if (!isWallInDirection(position.x, position.y, Heading::NORTH) && 
            position.y + 1 < MAZE_SIZE && 
            distances[position.x][position.y + 1] < leastDistance) {
            leastDistance = distances[position.x][position.y + 1];
            optimalMove = Action::RIGHT;
        }
        // Check left (South)
        if (!isWallInDirection(position.x, position.y, Heading::SOUTH) && 
            position.y - 1 >= 0 && 
            distances[position.x][position.y - 1] < leastDistance) {
            leastDistance = distances[position.x][position.y - 1];
            optimalMove = Action::LEFT;
        }
    }

    // Handle dead ends
    if (leastDistance == std::numeric_limits<unsigned int>::max()) {
        optimalMove = Action::RIGHT;
        API::debugLog("Dead end detected");
    }
    API::debugLog("Least distance: " + std::to_string(leastDistance));
    return optimalMove;
}

// [Rest of the code remains the same]

Action MazeSolver::leftWallFollower() {
    if (API::wallFront()) {
        if (API::wallLeft()) {
            return Action::RIGHT;
        }
        return Action::LEFT;
    }
    return Action::FORWARD;
}