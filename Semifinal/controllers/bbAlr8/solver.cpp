#include "solver.hpp"
#include "API.hpp"
#include <bitset>
#include <limits>
#include <queue>
#include <algorithm>
#include <iomanip>
#include <iostream>

// Define ANSI color codes for terminal output
#define ANSI_COLOR_RESET   "\033[0m"
#define ANSI_COLOR_GREEN   "\033[32m"
#define ANSI_COLOR_RED     "\033[31m"
#define ANSI_COLOR_BLUE    "\033[34m"

MazeSolver::MazeSolver()
    : maze{}, distances{}, position{10, 0}, heading{Heading::NORTH}, reachedCenter{false}
{
    initialize();
}

void MazeSolver::initialize()
{
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

    resetDistances({0,0});

    // Initialize visitCount with zeros
    for (int x = 0; x < MAZE_SIZE; ++x) {
        for (int y = 0; y < MAZE_SIZE; ++y) {
            visitCount[x][y] = 0;
        }
    }
    dfsStack.push(position); // Initial position
    backtracking = false;
    previousPosition = position;
}

void MazeSolver::updateMaze()
{
    int x = position.x;
    int y = position.y;
    unsigned int walls = WallConfig::NONE;

    switch (heading)
    {
    case Heading::NORTH:
        if (API_wallFront())
        {
            walls |= WallConfig::N;
            if (y + 1 != MAZE_SIZE)
                maze[x][y + 1] |= WallConfig::S;
        }
        if (API_wallLeft())
        {
            walls |= WallConfig::W;
            if (x - 1 >= 0)
                maze[x - 1][y] |= WallConfig::E;
        }
        if (API_wallRight())
        {
            walls |= WallConfig::E;
            if (x + 1 != MAZE_SIZE)
                maze[x + 1][y] |= WallConfig::W;
        }
        break;
    case Heading::EAST:
        if (API_wallFront())
        {
            walls |= WallConfig::E;
            if (x + 1 != MAZE_SIZE)
                maze[x + 1][y] |= WallConfig::W;
        }
        if (API_wallLeft())
        {
            walls |= WallConfig::N;
            if (y + 1 != MAZE_SIZE)
                maze[x][y + 1] |= WallConfig::S;
        }
        if (API_wallRight())
        {
            walls |= WallConfig::S;
            if (y - 1 >= 0)
                maze[x][y - 1] |= WallConfig::N;
        }
        break;
    case Heading::SOUTH:
        if (API_wallFront())
        {
            walls |= WallConfig::S;
            if (y - 1 >= 0)
                maze[x][y - 1] |= WallConfig::N;
        }
        if (API_wallLeft())
        {
            walls |= WallConfig::E;
            if (x + 1 != MAZE_SIZE)
                maze[x + 1][y] |= WallConfig::W;
        }
        if (API_wallRight())
        {
            walls |= WallConfig::W;
            if (x - 1 >= 0)
                maze[x - 1][y] |= WallConfig::E;
        }
        break;
    case Heading::WEST:
        if (API_wallFront())
        {
            walls |= WallConfig::W;
            if (x - 1 >= 0)
                maze[x - 1][y] |= WallConfig::E;
        }
        if (API_wallLeft())
        {
            walls |= WallConfig::S;
            if (y - 1 >= 0)
                maze[x][y - 1] |= WallConfig::N;
        }
        if (API_wallRight())
        {
            walls |= WallConfig::N;
            if (y + 1 != MAZE_SIZE)
                maze[x][y + 1] |= WallConfig::S;
        }
        break;
    }

    maze[x][y] |= walls;
    std::cout << "x : " << x << "  y : " << y << "  " << std::bitset<4>(maze[x][y]) << "  ";
}

int MazeSolver::xyToSquare(int x, int y) const
{
    return x + MAZE_SIZE * y;
}

Coordinate MazeSolver::squareToCoord(int square) const
{
    return {square % MAZE_SIZE, square / MAZE_SIZE};
}

void MazeSolver::resetDistances(Coordinate target)
{
    // Initially set all distances to -1 (invalid distance)
    for (auto &row : distances)
    {
        row.fill(-1);
    }

    // Set goal distances based on the target coordinate
    distances[target.x][target.y] = 0;
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
    resetDistances(targetCoordinate);
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

Action MazeSolver::solve()
{
    // Check if center reached or returned to start
    if (position.x == targetCoordinate.x && position.y == targetCoordinate.y)
    {
        return Action::IDLE;
    }

    //updateMaze();
    updateDistances();

    Action action = floodFill();
    updateHeading(action);
    updatePosition(action);
    return action;
}

bool MazeSolver::shouldMarkVisited(int x, int y) {
    int visitedNeighbors = 0;
    int totalNeighbors = 4;

    // Check North
    if (y + 1 < MAZE_SIZE) {
        if (visitCount[x][y + 1] > 0) visitedNeighbors++;
    } else {
        totalNeighbors--; // Edge case: No neighbor in this direction
    }

    // Check South
    if (y - 1 >= 0) {
        if (visitCount[x][y - 1] > 0) visitedNeighbors++;
    } else {
        totalNeighbors--;
    }

    // Check East
    if (x + 1 < MAZE_SIZE) {
        if (visitCount[x + 1][y] > 0) visitedNeighbors++;
    } else {
        totalNeighbors--;
    }

    // Check West
    if (x - 1 >= 0) {
        if (visitCount[x - 1][y] > 0) visitedNeighbors++;
    } else {
        totalNeighbors--;
    }

    // If all possible neighbors are visited, mark the cell as visited
    return (visitedNeighbors == totalNeighbors);
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

void MazeSolver::updateColour()
{
    int8_t color = API_getColour();
    int x = position.x;
    int y = position.y;

    if (color == 3)//if red found, fire pit is  drawn
    {
        addDangerZone(x, y);
    } else if (color == 2){
        API_add_OrangeNode(x, y);
    }

    API_detectAndAddSurvivor(x , y);
}

// Action MazeSolver::explore()
// {

//     if (allCellsExplored())
//     {
//         // API::debugLog("All cells explored!");
//         return Action::ALLEXPLORED;
//     }

//     visitCount[position.x][position.y]++;

//     if(visitCount[position.x][position.y] == 1)
//     {
//         updateMaze();
//     }
   
//     //updateMaze();
//     updateDistances();
//     updateColour();
//     Action action = tremauxSearch();
//     updateHeading(action);
//     updatePosition(action);
//     printMaze();
//     return action;
// }

Action MazeSolver::explore() {
    if (allCellsExplored()) {
        return Action::ALLEXPLORED;
    }

    visitCount[position.x][position.y]++;
    
    // for (int x = 0; x < MAZE_SIZE; ++x) {
    //     for (int y = 0; y < MAZE_SIZE; ++y) {
    //         // Mark as visited if all adjacent necessary walls are visited
    //         if (shouldMarkVisited(x, y)) {
    //         visitCount[x][y] = 2; // Ensure it's marked as visited
    // }
    //     }
    // }

    

    if(visitCount[position.x][position.y] == 1)
    {
        updateMaze();
    }
    updateDistances();
    updateColour();
    
    Action action = dfsSearch(); // Call dfsSearch instead of tremauxSearch
    updateHeading(action);
    updatePosition(action);
    
    printMaze();
    return action;
}

Action MazeSolver::dfsSearch() {
    const int x = position.x;
    const int y = position.y;
    std::vector<std::pair<Action, int>> possibleActions; // Store (action, visitCount)

    // Check possible moves in priority order: Forward, Left, Right
    auto checkAndAddAction = [&](Heading dir, Action act, int nx, int ny) {
        if (!isWallInDirection(x, y, dir)) {
            possibleActions.emplace_back(act, visitCount[nx][ny]); // Store action with visit count
        }
    };

    switch (heading) {
        case Heading::NORTH:
            checkAndAddAction(Heading::NORTH, Action::FORWARD, x, y + 1);
            checkAndAddAction(Heading::WEST, Action::LEFT, x - 1, y);
            checkAndAddAction(Heading::EAST, Action::RIGHT, x + 1, y);
            break;
        case Heading::EAST:
            checkAndAddAction(Heading::EAST, Action::FORWARD, x + 1, y);
            checkAndAddAction(Heading::NORTH, Action::LEFT, x, y + 1);
            checkAndAddAction(Heading::SOUTH, Action::RIGHT, x, y - 1);
            break;
        case Heading::SOUTH:
            checkAndAddAction(Heading::SOUTH, Action::FORWARD, x, y - 1);
            checkAndAddAction(Heading::EAST, Action::LEFT, x + 1, y);
            checkAndAddAction(Heading::WEST, Action::RIGHT, x - 1, y);
            break;
        case Heading::WEST:
            checkAndAddAction(Heading::WEST, Action::FORWARD, x - 1, y);
            checkAndAddAction(Heading::SOUTH, Action::LEFT, x, y - 1);
            checkAndAddAction(Heading::NORTH, Action::RIGHT, x, y + 1);
            break;
    }

    // Sort actions by visit count (lower count preferred)
    std::sort(possibleActions.begin(), possibleActions.end(),
              [](const std::pair<Action, int>& a, const std::pair<Action, int>& b) {
                  return a.second < b.second;
              });

    // Prefer less-visited cells
    if (!possibleActions.empty()) {
        backtracking = false;
        dfsStack.push(position); // Push current position before moving
        return possibleActions.front().first; // Return the action with the least visit count
    }

    // Backtrack if no new cells
    if (!dfsStack.empty() && !backtracking) {
        previousPosition = dfsStack.top();
        dfsStack.pop();
        backtracking = true;
    }

    if (backtracking) {
        // Calculate direction to previous position
        int dx = previousPosition.x - x;
        int dy = previousPosition.y - y;
        Heading requiredHeading = Heading::NORTH;

        if (dx == 1) requiredHeading = Heading::EAST;
        else if (dx == -1) requiredHeading = Heading::WEST;
        else if (dy == 1) requiredHeading = Heading::NORTH;
        else if (dy == -1) requiredHeading = Heading::SOUTH;

        // Determine turn direction
        int currentIdx = static_cast<int>(heading);
        int requiredIdx = static_cast<int>(requiredHeading);
        int diff = (requiredIdx - currentIdx + 4) % 4;

        if (diff == 1) return Action::RIGHT;
        else if (diff == 3) return Action::LEFT;
        else if (diff == 2) { // 180 turn
            backtracking = false;
            return Action::RIGHT; // Turn right twice (simplified)
        }
        else return Action::FORWARD;
    }

    return Action::IDLE;
}

Action MazeSolver::tremauxSearch()
{
    unsigned int leastVisits = std::numeric_limits<unsigned int>::max();
    Action optimalMove = Action::IDLE;

    // visitCount[position.x][position.y]++;

    // char color = visitCount[position.x][position.y] == 1 ? 'G' : visitCount[position.x][position.y] == 2 ? 'O' : 'R';
    // API::setColor(position.x, position.y, color);

    if (explorationMode && allCellsExplored())
    {
        explorationMode = false;
        // API::debugLog("Switching to return mode - all cells visited");
    }

    if (heading == Heading::NORTH)
    {
        // Check forward (North)
        if (!isWallInDirection(position.x, position.y, Heading::NORTH) &&
            position.y + 1 < MAZE_SIZE &&
            visitCount[position.x][position.y + 1] < leastVisits)
        {
            leastVisits = visitCount[position.x][position.y + 1];
            optimalMove = Action::FORWARD;
            // API::debugLog("Debug Point" + std::to_string(leastVisits));
        }
        // Check right (East)
        if (!isWallInDirection(position.x, position.y, Heading::EAST) &&
            position.x + 1 < MAZE_SIZE &&
            visitCount[position.x + 1][position.y] < leastVisits)
        {
            leastVisits = visitCount[position.x + 1][position.y];
            optimalMove = Action::RIGHT;
        }
        // Check left (West)
        if (!isWallInDirection(position.x, position.y, Heading::WEST) &&
            position.x - 1 >= 0 &&
            visitCount[position.x - 1][position.y] < leastVisits)
        {
            leastVisits = visitCount[position.x - 1][position.y];
            optimalMove = Action::LEFT;
        }
    }
    else if (heading == Heading::EAST)
    {
        // Check forward (East)
        if (!isWallInDirection(position.x, position.y, Heading::EAST) &&
            position.x + 1 < MAZE_SIZE &&
            visitCount[position.x + 1][position.y] < leastVisits)
        {
            leastVisits = visitCount[position.x + 1][position.y];
            optimalMove = Action::FORWARD;
        }
        // Check right (South)
        if (!isWallInDirection(position.x, position.y, Heading::SOUTH) &&
            position.y - 1 >= 0 &&
            visitCount[position.x][position.y - 1] < leastVisits)
        {
            leastVisits = visitCount[position.x][position.y - 1];
            optimalMove = Action::RIGHT;
        }
        // Check left (North)
        if (!isWallInDirection(position.x, position.y, Heading::NORTH) &&
            position.y + 1 < MAZE_SIZE &&
            visitCount[position.x][position.y + 1] < leastVisits)
        {
            leastVisits = visitCount[position.x][position.y + 1];
            optimalMove = Action::LEFT;
        }
    }
    else if (heading == Heading::SOUTH)
    {
        // Check forward (South)
        if (!isWallInDirection(position.x, position.y, Heading::SOUTH) &&
            position.y - 1 >= 0 &&
            visitCount[position.x][position.y - 1] < leastVisits)
        {
            leastVisits = visitCount[position.x][position.y - 1];
            optimalMove = Action::FORWARD;
        }
        // Check right (West)
        if (!isWallInDirection(position.x, position.y, Heading::WEST) &&
            position.x - 1 >= 0 &&
            visitCount[position.x - 1][position.y] < leastVisits)
        {
            leastVisits = visitCount[position.x - 1][position.y];
            optimalMove = Action::RIGHT;
        }
        // Check left (East)
        if (!isWallInDirection(position.x, position.y, Heading::EAST) &&
            position.x + 1 < MAZE_SIZE &&
            visitCount[position.x + 1][position.y] < leastVisits)
        {
            leastVisits = visitCount[position.x + 1][position.y];
            optimalMove = Action::LEFT;
        }
    }
    else if (heading == Heading::WEST)
    {
        // Check forward (West)
        if (!isWallInDirection(position.x, position.y, Heading::WEST) &&
            position.x - 1 >= 0 &&
            visitCount[position.x - 1][position.y] < leastVisits)
        {
            leastVisits = visitCount[position.x - 1][position.y];
            optimalMove = Action::FORWARD;
        }
        // Check right (North)
        if (!isWallInDirection(position.x, position.y, Heading::NORTH) &&
            position.y + 1 < MAZE_SIZE &&
            visitCount[position.x][position.y + 1] < leastVisits)
        {
            leastVisits = visitCount[position.x][position.y + 1];
            optimalMove = Action::RIGHT;
        }
        // Check left (South)
        if (!isWallInDirection(position.x, position.y, Heading::SOUTH) &&
            position.y - 1 >= 0 &&
            visitCount[position.x][position.y - 1] < leastVisits)
        {
            leastVisits = visitCount[position.x][position.y - 1];
            optimalMove = Action::LEFT;
        }
    }
    // Handle dead ends
    if (leastVisits == std::numeric_limits<unsigned int>::max())
    {
        optimalMove = Action::RIGHT;
        std::cout << "dead end" << std::endl;
    }
    // if (optimalMove == Action::FORWARD)
    // {
    //     visitCount[position.x][position.y]++;
    // }

    return optimalMove;
}

Action MazeSolver::floodFill() const
{
    int leastDistance = std::numeric_limits<int>::max();
    Action optimalMove = Action::IDLE;

    // Check relative positions based on current heading
    if (heading == Heading::NORTH)
    {
        // Check forward (North)
        if (!isWallInDirection(position.x, position.y, Heading::NORTH) &&
            position.y + 1 < MAZE_SIZE &&
            distances[position.x][position.y + 1] < leastDistance)
        {
            leastDistance = distances[position.x][position.y + 1];
            optimalMove = Action::FORWARD;
        }
        // Check right (East)
        if (!isWallInDirection(position.x, position.y, Heading::EAST) &&
            position.x + 1 < MAZE_SIZE &&
            distances[position.x + 1][position.y] < leastDistance)
        {
            leastDistance = distances[position.x + 1][position.y];
            optimalMove = Action::RIGHT;
        }
        // Check left (West)
        if (!isWallInDirection(position.x, position.y, Heading::WEST) &&
            position.x - 1 >= 0 &&
            distances[position.x - 1][position.y] < leastDistance)
        {
            leastDistance = distances[position.x - 1][position.y];
            optimalMove = Action::LEFT;
        }
    }
    else if (heading == Heading::EAST)
    {
        // Check forward (East)
        if (!isWallInDirection(position.x, position.y, Heading::EAST) &&
            position.x + 1 < MAZE_SIZE &&
            distances[position.x + 1][position.y] < leastDistance)
        {
            leastDistance = distances[position.x + 1][position.y];
            optimalMove = Action::FORWARD;
        }
        // Check right (South)
        if (!isWallInDirection(position.x, position.y, Heading::SOUTH) &&
            position.y - 1 >= 0 &&
            distances[position.x][position.y - 1] < leastDistance)
        {
            leastDistance = distances[position.x][position.y - 1];
            optimalMove = Action::RIGHT;
        }
        // Check left (North)
        if (!isWallInDirection(position.x, position.y, Heading::NORTH) &&
            position.y + 1 < MAZE_SIZE &&
            distances[position.x][position.y + 1] < leastDistance)
        {
            leastDistance = distances[position.x][position.y + 1];
            optimalMove = Action::LEFT;
        }
    }
    else if (heading == Heading::SOUTH)
    {
        // Check forward (South)
        if (!isWallInDirection(position.x, position.y, Heading::SOUTH) &&
            position.y - 1 >= 0 &&
            distances[position.x][position.y - 1] < leastDistance)
        {
            leastDistance = distances[position.x][position.y - 1];
            optimalMove = Action::FORWARD;
        }
        // Check right (West)
        if (!isWallInDirection(position.x, position.y, Heading::WEST) &&
            position.x - 1 >= 0 &&
            distances[position.x - 1][position.y] < leastDistance)
        {
            leastDistance = distances[position.x - 1][position.y];
            optimalMove = Action::RIGHT;
        }
        // Check left (East)
        if (!isWallInDirection(position.x, position.y, Heading::EAST) &&
            position.x + 1 < MAZE_SIZE &&
            distances[position.x + 1][position.y] < leastDistance)
        {
            leastDistance = distances[position.x + 1][position.y];
            optimalMove = Action::LEFT;
        }
    }
    else if (heading == Heading::WEST)
    {
        // Check forward (West)
        if (!isWallInDirection(position.x, position.y, Heading::WEST) &&
            position.x - 1 >= 0 &&
            distances[position.x - 1][position.y] < leastDistance)
        {
            leastDistance = distances[position.x - 1][position.y];
            optimalMove = Action::FORWARD;
        }
        // Check right (North)
        if (!isWallInDirection(position.x, position.y, Heading::NORTH) &&
            position.y + 1 < MAZE_SIZE &&
            distances[position.x][position.y + 1] < leastDistance)
        {
            leastDistance = distances[position.x][position.y + 1];
            optimalMove = Action::RIGHT;
        }
        // Check left (South)
        if (!isWallInDirection(position.x, position.y, Heading::SOUTH) &&
            position.y - 1 >= 0 &&
            distances[position.x][position.y - 1] < leastDistance)
        {
            leastDistance = distances[position.x][position.y - 1];
            optimalMove = Action::LEFT;
        }
    }

    // Handle dead ends
    if (leastDistance == std::numeric_limits<int>::max())
    {
        optimalMove = Action::RIGHT;
        // API::debugLog("Dead end detected");
    }
    // API::debugLog("Least distance: " + std::to_string(leastDistance));
    return optimalMove;
}

void MazeSolver::addDangerZone(int x, int y)
{
    if (x < 0 || x + 4 >= MAZE_SIZE || y < 0 || y + 4 >= MAZE_SIZE)
    {
        std::cerr << "Danger zone out of bounds at (" << x << ", " << y << ")\n";
        return;
    }

    // Mark center cell (1x1)
    API_add_RedNode(x, y); //this need to come before adding orange cells or this will be also added  to the orange cells along with the red node

    // Mark inner layer (3x3)
    for (int i = x - 1; i < x + 2; i++)
    {
        for (int j = y - 1; j < y + 2; j++)
        {
            API_add_OrangeNode(i, j);
        }
    }
}

void MazeSolver::printMaze() const
{
    // Print column numbers
    std::cout << "   ";
    for (int x = 0; x < MAZE_SIZE; ++x)
    {
        std::cout << std::setw(3) << x;
    }
    std::cout << "\n";

    // For each row
    for (int y = MAZE_SIZE - 1; y >= 0; --y)
    {
        // Print row number
        std::cout << std::setw(2) << y << " ";

        // Print top walls for each cell in the row
        for (int x = 0; x < MAZE_SIZE; ++x)
        {
            std::cout << "+";
            if (maze[x][y] & WallConfig::N)
            {
                std::cout << "---";
            }
            else
            {
                std::cout << "   ";
            }
        }
        std::cout << "+\n";

        // Print row number again
        std::cout << "   ";

        // Print side walls and cell contents
        for (int x = 0; x < MAZE_SIZE; ++x)
        {
            if (maze[x][y] & WallConfig::W)
            {
                std::cout << "|";
            }
            else
            {
                std::cout << " ";
            }

            // Print cell content
            if (x == position.x && y == position.y)
            {
                // Print direction the robot is facing
                switch (heading)
                {
                case Heading::NORTH:
                    std::cout << "^";
                    break;
                case Heading::EAST:
                    std::cout << ">";
                    break;
                case Heading::SOUTH:
                    std::cout << "v";
                    break;
                case Heading::WEST:
                    std::cout << "<";
                    break;
                }
                // Print distance value after direction
                if (distances[x][y] >= 0)
                {
                    std::cout << std::setw(2) << distances[x][y];
                }
                else
                {
                    std::cout << " ?";
                }
            }
            else
            {
                // Print just the distance for non-robot cells
                if (distances[x][y] >= 0)
                {
                    // Check if the cell has been visited
                    if (visitCount[x][y] > 0)
                    {
                        // Visited cell: print distance in green
                        std::cout << ANSI_COLOR_GREEN << std::setw(3) << distances[x][y] << ANSI_COLOR_RESET;
                    }
                    else
                    {
                        // Unvisited cell: print distance in default color
                        std::cout << std::setw(3) << distances[x][y];
                    }
                }
                else
                {
                    std::cout << " ? ";
                }
            }
        }

        // Print final wall of row
        if (maze[MAZE_SIZE - 1][y] & WallConfig::E)
        {
            std::cout << "|\n";
        }
        else
        {
            std::cout << " \n";
        }
    }

    // Print bottom row of walls
    std::cout << "   ";
    for (int x = 0; x < MAZE_SIZE; ++x)
    {
        std::cout << "+";
        if (maze[x][0] & WallConfig::S)
        {
            std::cout << "---";
        }
        else
        {
            std::cout << "   ";
        }
    }
    std::cout << "+\n";

    // Print legend
    std::cout << "\nLegend:\n";
    std::cout << "^,>,v,< - Robot direction\n";
    std::cout << "Numbers - Distance from target\n";
    std::cout << ANSI_COLOR_GREEN << "Green" << ANSI_COLOR_RESET << " - Visited cell\n";
    std::cout << "?       - Unexplored/unreachable\n";
    std::cout << "---|    - Walls\n";
}